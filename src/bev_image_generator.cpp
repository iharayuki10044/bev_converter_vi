#include "bev_converter/bev_image_generator.h"


BEVImageGenerator::BEVImageGenerator(double range, int grid_num, int manual_crop_size, XmlRpc::XmlRpcValue robot_param, std::string frame_id, std::string child_frame_id)
: nh("~")
{
    RANGE = range;
    GRID_NUM = grid_num;
	MANUAL_CROP_SIZE = manual_crop_size;
	crop_size = manual_crop_size;
	ROBOT_PARAM = robot_param;
	FRAME_ID = frame_id;
	CHILD_FRAME_ID = child_frame_id;
    // nh.param("");
}


cv::Mat BEVImageGenerator::cropped_current_grid_img_generator(cv::Mat src_img)
{
	/* std::cout << "BEVImageGenerator::cropped_current_grid_img_generator" << std::endl; */
    cv::Mat dst_img = image_cropper(src_img);

    return dst_img;
}


cv::Mat BEVImageGenerator::cropped_transformed_grid_img_generator(cv::Mat src_img, Eigen::Vector3d current_position, double current_yaw, Eigen::Vector3d last_position, double last_yaw)
{
	/* std::cout << "BEVImageGenerator::cropped_transformed_grid_img_generator" << std::endl; */
	double d_yaw = current_yaw - last_yaw;
	d_yaw = atan2(sin(d_yaw), cos(d_yaw));
	Eigen::Matrix3d r;
	r = Eigen::AngleAxisd(-d_yaw, Eigen::Vector3d::UnitZ());

	Eigen::Matrix3d last_yaw_rotation;
	last_yaw_rotation = Eigen::AngleAxisd(-last_yaw, Eigen::Vector3d::UnitZ());
	Eigen::Vector3d _current_position = last_yaw_rotation * current_position;
	Eigen::Vector3d _last_position = last_yaw_rotation * last_position;
	Eigen::Translation<double, 3> t(_last_position - _current_position);

	affine_transform = t * r;
	/* std::cout << "affine transformation: \n" << affine_transform.translation() << "\n" << affine_transform.rotation().eulerAngles(0,1,2) << std::endl; */
	
    cv::Mat transformed_grid_img, dst_img;

	transformed_grid_img = image_transformer(src_img);
	dst_img = image_cropper(transformed_grid_img);
	odom_callback_flag = false;

    return dst_img;
}


void BEVImageGenerator::formatter(void)
{
	/* std::cout << "BEVImageGenerator::formatter" << std::endl; */

    dt = 1.0 / Hz;
	elapsed_time = 0.0;
    grid_size = RANGE / GRID_NUM;

	src_euqlid_3pts.points.resize(0);
	pt0.x = 0.0;
	pt0.y = 0.0;
	pt0.z = 0.0;
	pt1.x = 0.5 * RANGE;
	pt1.y = 0.0;
	pt1.z = 0.0;
	pt2.x = 0.0;
	pt2.y = 0.5 * RANGE;
	pt2.z = 0.0;
	src_euqlid_3pts.points.push_back(pt0);
	src_euqlid_3pts.points.push_back(pt1);
	src_euqlid_3pts.points.push_back(pt2);
}


void BEVImageGenerator::initializer(void)
{
	/* std::cout << "BEVImageGenerator::initializer" << std::endl; */
}


cv::Mat BEVImageGenerator::image_transformer(cv::Mat src_img)
{
	std::cout << "BEVImageGenerator::image_transformer" << std::endl;

	float alpha = 1.0;
    pcl::transformPointCloud(src_euqlid_3pts, dst_euqlid_3pts, affine_transform);

	const cv::Point2f src_pt[] = {cv::Point2f(src_euqlid_3pts.points[0].x / grid_size, src_euqlid_3pts.points[0].y / grid_size),
								  cv::Point2f(src_euqlid_3pts.points[1].x / grid_size, src_euqlid_3pts.points[1].y / grid_size),
								  cv::Point2f(src_euqlid_3pts.points[2].x / grid_size, src_euqlid_3pts.points[2].y / grid_size)};
	/* const cv::Point2f dst_pt[] = {cv::Point2f(alpha * dst_euqlid_3pts.points[0].x / grid_size, alpha * dst_euqlid_3pts.points[0].y / grid_size), */
	/* 							  cv::Point2f(alpha * dst_euqlid_3pts.points[1].x / grid_size, alpha * dst_euqlid_3pts.points[1].y / grid_size), */
	/* 							  cv::Point2f(alpha * dst_euqlid_3pts.points[2].x / grid_size, alpha * dst_euqlid_3pts.points[2].y / grid_size)}; */

	const cv::Point2f dst_pt[] = {cv::Point2f(-alpha * dst_euqlid_3pts.points[0].y / grid_size, -alpha * dst_euqlid_3pts.points[0].x / grid_size),
								  cv::Point2f(-alpha * dst_euqlid_3pts.points[1].x / grid_size, -alpha * dst_euqlid_3pts.points[1].x / grid_size),
								  cv::Point2f(-alpha * dst_euqlid_3pts.points[2].x / grid_size, -alpha * dst_euqlid_3pts.points[2].x / grid_size)};
	/* const cv::Point2f src_pt[] = {cv::Point2f(src_euqlid_3pts.points[0].x, src_euqlid_3pts.points[0].y), */
	/* 							  cv::Point2f(src_euqlid_3pts.points[1].x, src_euqlid_3pts.points[1].y), */
	/* 							  cv::Point2f(src_euqlid_3pts.points[2].x, src_euqlid_3pts.points[2].y)}; */
	/* const cv::Point2f dst_pt[] = {cv::Point2f(dst_euqlid_3pts.points[0].x, dst_euqlid_3pts.points[0].y), */
	/* 							  cv::Point2f(dst_euqlid_3pts.points[1].x, dst_euqlid_3pts.points[1].y), */
	/* 							  cv::Point2f(dst_euqlid_3pts.points[2].x, dst_euqlid_3pts.points[2].y)}; */
    const cv::Mat affine_matrix = cv::getAffineTransform(src_pt, dst_pt);
    cv::Mat dst_img;
    cv::warpAffine(src_img, dst_img, affine_matrix, src_img.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

	std::cout << "src_pt" << std::endl;
	for(int i = 0; i < 3; i++){
		std::cout << src_pt[i] << std::endl;
	}
	std::cout << "dst_pt" << std::endl;
	for(int i = 0; i < 3; i++){
		std::cout << dst_pt[i] << std::endl;
	}
    return dst_img;
}


cv::Mat BEVImageGenerator::image_cropper(cv::Mat src_img)
{
	/* std::cout << "BEVImageGenerator::image_cropper" << std::endl; */

    cv::Rect roi(cv::Point(crop_size, crop_size), cv::Size(GRID_NUM - 2 * crop_size, GRID_NUM - 2 * crop_size));
    cv::Mat dst_img = src_img(roi);

    return dst_img;
}



