// g2o - General Graph Optimization
// Copyright (C) 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include"header.h"

//#define _POSE_SENSOR_OFFSET
//#define _POSE_PRIOR_SENSOR

using namespace g2o;
using namespace std;
using namespace Eigen;

void file_reader(string file) {
	std::cout << "file path:" << file << std::endl;
	std::ifstream ifs_pose_info(file, std::ios::in);
	if (!ifs_pose_info)
		std::cout << "error:robot_pose" << std::endl;	// File open failed

	std::string buf_pose;
	std::getline(ifs_pose_info, buf_pose);
	//std::cout << buf_pose << std::endl;
	int count = 1;//scanを省くため
	int num = 0;
	while (ifs_pose_info && std::getline(ifs_pose_info, buf_pose)) {
		std::vector<std::string> v;
		boost::algorithm::split(v, buf_pose, boost::is_any_of(","));
		if (v.size() < 37)
			continue;
		for (int k = 0; k < 6; k++) {
			for (int l = 0; l < 6; l++) {
				if (k == l) {
					info_mat[num](k, l) = abs(std::stod(v[count].c_str()))*100;
					//info_mat[num](k, l) = std::stod(v[count].c_str());
				}
				count++;
			}
		}
		count = 1;
		num++;
	}
}

void file_reader_relative(string file) {
	std::cout << "file path:" << file << std::endl;
	std::ifstream ifs_file(file, std::ios::in);
	if (!ifs_file) {
		std::cout << "error  :   file" << std::endl;	// File open failed
		getchar();
	}
	int count = 2;//input_scan/target_scannを省くため
	int num = 0;
	std::string buf;
	std::getline(ifs_file, buf); //入力ストリームから文字列を行単位で排除
	//cout << buf << endl;
	while (ifs_file && std::getline(ifs_file, buf)) {
		std::vector<std::string> v;
		boost::algorithm::split(v, buf, boost::is_any_of(","));
		if (v.size() < 38) {
			continue;
			getchar();
		}

		//cout << "scan:" << std::stoi(v[0].c_str()) << endl;
		int target_scan = std::stoi(v[0].c_str());
		int input_scan = std::stoi(v[1].c_str());
		for (int k = 0; k < 6; k++) {
			for (int l = 0; l < 6; l++) {
				if (k == l) {
					info_relative_mat[num](k, l) = abs(std::stod(v[count].c_str()));
					//relative_info_mat[num](k, l) = std::stod(v[count].c_str());
				}
				count++;
			}
		}
		count = 2;
		num++;
	}
}

int main(int argc, char** argv) {
	_mkdir("Output");
	string outputFilename = "Output/Pose_graph.g2o";
	string outputFilenameOptimized = "Output/Pose_graph_optimized.g2o";
	const std::string file_pose = "C:/Databox/三山木/三山木3/吉田ベースプログラム結果/180~950and8500~8800/すれ違い/180~870/Optimized_pose.csv";	//ロボットの姿勢
	const std::string file_pose_info = "C:/Databox/三山木/三山木3/吉田ベースプログラム結果/180~950and8500~8800/すれ違い/180~870/Information_matrix.csv";	//情報行列
	//const std::string file_relative_pose = "C:/Databox/三山木/三山木3/180~950and7400~8800/180~950/relative_pose.csv";	//相対姿勢(制約)
	//const std::string file_relative_pose_info = "C:/Databox/三山木/三山木3/180~950and7400~8800/180~950/Information_matrix.csv";	////情報行列

	current_pose.Zero();
	pre_pose.Zero();
	constraints_pose.Zero();
	offset_pose.Zero();
	file_reader(file_pose_info);
	//file_reader_relative(file_relative_pose_info);

	// create the linear solver
	BlockSolverX::LinearSolverType * linearSolver = new LinearSolverCSparse<BlockSolverX::PoseMatrixType>();

	// create the block solver on top of the linear solver
	BlockSolverX* blockSolver = new BlockSolverX(linearSolver);

	// create the algorithm to carry out the optimization
	OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);
	// NOTE: We skip to fix a variable here, either this is stored in the file
	// itself or Levenberg will handle it.

	// create the optimizer to load the data and carry out the optimization
	SparseOptimizer optimizer;

	optimizer.setAlgorithm(optimizationAlgorithm);

	//Add parameter offset
	ParameterSE3Offset* odomOffset = new ParameterSE3Offset();
	odomOffset->setId(0);
	optimizer.addParameter(odomOffset);

	int vertex_id = 0;
	g2o::VertexSE3 *v_se3_first = new g2o::VertexSE3;

	//姿勢情報の読み込み
	std::cout << "file path:" << file_pose << std::endl;
	std::ifstream ifs_pose(file_pose, std::ios::in);
	if (!ifs_pose)
		std::cout << "error:robot_pose" << std::endl;	// File open failed

	std::string buf_pose;
	std::getline(ifs_pose, buf_pose);
	std::cout << buf_pose << std::endl;
	static int main_count = 0;
	while (ifs_pose && std::getline(ifs_pose, buf_pose)) {
		std::vector<std::string> v;
		boost::algorithm::split(v, buf_pose, boost::is_any_of(","));
		if (v.size() < 7)
			continue;

		//ファイルから姿勢情報を入力
		current_pose(6) = std::atoi(v[0].c_str());
		current_pose(0) = std::stod(v[1].c_str());
		current_pose(1) = std::stod(v[2].c_str());
		current_pose(2) = std::stod(v[3].c_str());
		current_pose(3) = std::stod(v[4].c_str());
		current_pose(4) = std::stod(v[5].c_str());
		current_pose(5) = std::stod(v[6].c_str());

		/*current_pose(3) = current_pose(3) / 180 * M_PI;
		current_pose(4) = current_pose(4) / 180 * M_PI;
		current_pose(5) = current_pose(5) / 180 * M_PI;*/
		//Angle +value
		if (current_pose(3) < 0) {
			current_pose(3) = 2.0 * M_PI + current_pose(3);
		}
		if (current_pose(4) < 0) {
			current_pose(4) = 2.0 * M_PI + current_pose(4);
		}

		if (current_pose(5) < 0) {
			current_pose(5) = 2.0 * M_PI + current_pose(5);
		}

		//初期位置を0に設定
		/*if (main_count == 0) {
			current_pose(0) = 0;
			current_pose(1) = 0;
			current_pose(2) = 0;
			current_pose(3) = 0;
			current_pose(4) = 0;
			current_pose(5) = 0;
			scan_start_id = current_pose(6);
		}*/

		//***グラフにノードを追加 ここから***//
		//位置情報の設定(x,y,z)
		/*Eigen::Vector3d trans;*/
		trans(0) = current_pose(0);
		trans(1) = current_pose(1);
		trans(2) = current_pose(2);

		//姿勢情報をクォータニオンで設定(roll,pitch,yaw)
		Eigen::Quaterniond q;
		/*q = Eigen::AngleAxisd(current_pose(3), Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(current_pose(4), Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(current_pose(5), Eigen::Vector3d::UnitZ());*/

		q = Eigen::AngleAxisd(current_pose(5), Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(current_pose(4), Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(current_pose(3), Eigen::Vector3d::UnitX());


			//グラフにノード追加
		g2o::SE3Quat pose(q, trans);
		g2o::VertexSE3 *v_se3 = new g2o::VertexSE3;
		v_se3->setId(current_pose(6));

		if (main_count == 0)
		{
			v_se3->setFixed(true);
			v_se3_first = v_se3;
			info_mat[main_count].Identity();
		}
		v_se3->setEstimate(pose);
		//***グラフにノードを追加 ここまで***//

		if (!optimizer.addVertex(v_se3)) {
			std::cout << "Failing add Vertex" << current_pose(6) << std::endl;
			getchar();
		}
		else {
			std::cout << "Success add Vertex:" << current_pose(6) << std::endl;
		}

		if (main_count > 0)
		{
			g2o::EdgeSE3 *e = new g2o::EdgeSE3();
			e->setVertex(0, optimizer.vertex(pre_pose(6)));
			e->setVertex(1, optimizer.vertex(current_pose(6)));
			g2o::Isometry3D iso;
			iso = Isometry3D::Identity();

			offset_pose = current_pose - pre_pose;

			if (offset_pose(3) < 0)
			{
				offset_pose(3) = 2.0*M_PI + offset_pose(3);
			}
			if (offset_pose(4) < 0)
			{
				offset_pose(4) = 2.0*M_PI + offset_pose(4);
			}
			if (offset_pose(5) < 0)
			{
				offset_pose(5) = 2.0*M_PI + offset_pose(5);
			}
			iso = Eigen::Translation3d(offset_pose(0), offset_pose(1), offset_pose(2))
				* Eigen::AngleAxisd(offset_pose(3), Eigen::Vector3d::UnitX())
				* Eigen::AngleAxisd(offset_pose(4), Eigen::Vector3d::UnitY())
				* Eigen::AngleAxisd(offset_pose(5), Eigen::Vector3d::UnitZ());

			/*cov_matrix.setZero();
			cov_matrix(0, 0) = 0.02*0.02;
			cov_matrix(1, 1) = 0.02*0.02;
			cov_matrix(2, 2) = 0.04*0.04;
			cov_matrix(3, 3) = 0.01*0.01;
			cov_matrix(4, 4) = 0.01*0.01;
			cov_matrix(5, 5) = 0.01*0.01;
			Matrix6d info_mat = cov_matrix.inverse();*/

			e->setMeasurementFromState();	//ノード間の相対姿勢を追加
			e->setInformation(info_mat[main_count]);	//不確実性(情報行列形式:共分散行列の逆行列)
			optimizer.addEdge(e);
		}

		pre_pose = current_pose;
		main_count++;
	}


	//ループ検出結果による制約を与える
	//std::cout << "file path:" << file_relative_pose << std::endl;
	//std::ifstream ifs_relative(file_relative_pose, std::ios::in);
	//if (!ifs_relative)
	//	std::cout << "error:relative" << std::endl;	// File open failed

	//std::string buf_relative;
	//std::getline(ifs_relative, buf_relative);
	//std::cout << buf_relative << std::endl;
	//int rel_count = 0;

	//while (ifs_relative && std::getline(ifs_relative, buf_relative)) {
	//	std::vector<std::string> v_r;
	//	boost::algorithm::split(v_r, buf_relative, boost::is_any_of(","));
	//	if (v_r.size() < 9) {
	//		std::cout << v_r.size() << std::endl;
	//		continue;
	//	}
	//	int edge_to = std::atoi(v_r[0].c_str());
	//	int edge_from = std::atoi(v_r[1].c_str());
	//	double relative_score = std::stod(v_r[2].c_str());
	//	double relative_x = std::stod(v_r[3].c_str());
	//	double relative_y = std::stod(v_r[4].c_str());
	//	double relative_z = std::stod(v_r[5].c_str());
	//	double relative_roll = std::stod(v_r[6].c_str());
	//	double relative_pitch = std::stod(v_r[7].c_str());
	//	double relative_yaw = std::stod(v_r[8].c_str());

	//	if (relative_score > score_threshold) {
	//		continue;
	//	}

	//	/*cov_matrix.setZero();
	//	cov_matrix(0, 0) = 0.02*0.02;
	//	cov_matrix(1, 1) = 0.02*0.02;
	//	cov_matrix(2, 2) = 0.04*0.04;
	//	cov_matrix(3, 3) = 0.01*0.01;
	//	cov_matrix(4, 4) = 0.01*0.01;
	//	cov_matrix(5, 5) = 0.01*0.01;
	//	info_mat.Zero();
	//	info_mat = cov_matrix.inverse();*/

	//	g2o::EdgeSE3 *e = new g2o::EdgeSE3();

	//	e->setVertex(0, optimizer.vertex(edge_from));
	//	e->setVertex(1, optimizer.vertex(edge_to));

	//	trans(0) = relative_x;
	//	trans(1) = relative_y;
	//	trans(2) = relative_z;

	//	//姿勢情報をクォータニオンで設定(roll,pitch,yaw)
	//	Eigen::Quaterniond q;
	//	/*q = Eigen::AngleAxisd(relative_roll, Eigen::Vector3d::UnitX())
	//		* Eigen::AngleAxisd(relative_pitch, Eigen::Vector3d::UnitY())
	//		* Eigen::AngleAxisd(relative_yaw, Eigen::Vector3d::UnitZ());*/

	//	q = Eigen::AngleAxisd(relative_yaw, Eigen::Vector3d::UnitZ())
	//		* Eigen::AngleAxisd(relative_pitch, Eigen::Vector3d::UnitY())
	//		* Eigen::AngleAxisd(relative_roll, Eigen::Vector3d::UnitX());

	//	//グラフにノード追加
	//	g2o::SE3Quat relative_pose(q, trans);
	//	
	//	e->setMeasurement(relative_pose.inverse());	//ノード間の相対姿勢
	//	e->setInformation(info_relative_mat[rel_count]);	//不確実性(情報行列)
	//	optimizer.addEdge(e);

	//	rel_count++;
	//}


	std::cout << "optimizer_size:" << optimizer.vertices().size() << std::endl;

	if (outputFilename.size() > 0) {
		if (outputFilename == "-") {
			cerr << "saving to stdout";
			optimizer.save(cout);
		}
		else {
			cerr << "saving " << outputFilename << " ... ";
			optimizer.save(outputFilename.c_str());
		}
		cerr << "done." << endl;
	}

	optimizer.initializeOptimization();
	optimizer.optimize(maxIterations);

	if (outputFilenameOptimized.size() > 0) {
		if (outputFilenameOptimized == "-") {
			cerr << "saving to stdout";
			optimizer.save(cout);
		}
		else {
			cerr << "saving " << outputFilenameOptimized << " ... ";
			optimizer.save(outputFilenameOptimized.c_str());
		}
		cerr << "done." << endl;
	}

	return 0;
}
