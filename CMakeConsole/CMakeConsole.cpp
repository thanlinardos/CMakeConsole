#include "CMakeConsole.h"

#define DIMENSIONS 3
#define AUTOPLAY false
#define SEC_PER_FLOOR 10
#define GND_WIDTH 40
#define GND_TOP 10
#define GND_BOTTOM -10
#define GND_DEPTH 10
#include <chrono>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream> 
using namespace std;
using namespace vvr;
using namespace Eigen;
using namespace math;
string PROJECT_ROOT = "C:/Users/thanl/Documents/CPP_VS22/CMakeConsole";
Mesh3DScene::Mesh3DScene()
{
	//! Load settings.
	vvr::Shape::DEF_LINE_WIDTH = 4;
	vvr::Shape::DEF_POINT_SIZE = 10;
	m_perspective_proj = true;
	m_hide_log = false;
	m_hide_sliders = false;
	m_fullscreen = false;
	m_bg_col = Colour("768E77");
	m_obj_col = Colour("454545");
	if (AUTOPLAY) m_anim.update(true);
	m_KDTree = NULL;
	temp_KDTree = NULL;
	reset();
}

void Mesh3DScene::reset()
{
	Scene::reset();

	//! Define plane
	m_plane_d = 0;
	m_plane = Plane(vec(0, 1, 1).Normalized(), m_plane_d);

	//! Define what will be visible by default
	m_style_flag = 0;
	m_style_flag |= FLAG(SHOW_AXES);
	//m_style_flag |= FLAG(SHOW_FPS);
	//m_style_flag |= FLAG(SHOW_PLANE);
	m_style_flag |= FLAG(SHOW_WIRE);
	//m_style_flag |= FLAG(SHOW_NORMALS);
	//m_style_flag |= FLAG(SHOW_SOLID);
	//m_style_flag |= FLAG(RUN_PREP);
	//m_style_flag |= FLAG(RUN_A1);
	//m_style_flag |= FLAG(RUN_A2);
	//m_style_flag |= FLAG(RUN_A3);
	//m_style_flag |= FLAG(RUN_B1);
	//m_style_flag |= FLAG(SHOW_B1);
	//Create Mesh
	string objDir = PROJECT_ROOT+"/CMakeConsole/resources/obj/";
	//std::cout << getBasePath();
	fstream objList;
	objList.open(objDir + "list.txt");
	file_paths.clear();
	obj_names.clear();
	std::cout << "choose obj file:\n";
	int i = 1;
	string str;
	if (objList.is_open()) {
		while (getline(objList,str))
		{
			string str2 = objDir + str + ".obj";
			file_paths.push_back(str2);
			obj_names.push_back(str);
			std::cout << "(" << i << ")" << str << ", ";
			i++;
		}
	}
	std::cout << endl;
	cin >> objNo;
	std::cout << "Current Object:" << obj_names[objNo - 1] << endl;
	objFile = file_paths[objNo - 1];
	m_model_original = vvr::Mesh(objFile);
	boundary = m_model_original.getAABB();
	//cubeFile = file_paths[5];
	cout << "Choose second object:" << endl;
	cin >> tempNo;
	cubeFile = file_paths[tempNo - 1];
	temp_B1_original = vvr::Mesh(cubeFile);
	//! Reset animation
	m_anim.setTime(0);
	//Reset Meshes
	temp_A1 = Mesh();
	temp_A2 = Mesh();
	temp_A3 = Mesh();
	temp_B1 = Mesh();
	ground = Grounds();
	delete m_KDTree;
	delete temp_KDTree;

	//remove dup vertices
	m_model_non_dup = vvr::Mesh(objFile);
	temp_B1_non_dup = vvr::Mesh(cubeFile);
	cout << "l1=" << m_model_original.getVertices().size() << endl;
	remove_dups(m_model_original.getVertices(), m_model_original.getTriangles());
	cout << "l2=" << m_model_original.getVertices().size() << endl;
	m_model_original.update();
	cout << "l1t=" << temp_B1_original.getVertices().size() << endl;
	//remove_dups(temp_B1_original.getVertices(), temp_B1_original.getTriangles());
	cout << "l2t=" << temp_B1_original.getVertices().size() << endl;
	temp_B1_original.update();
	vector<vec>& vertices = m_model_original.getVertices();
	vector<vec>& t_vertices = temp_B1_original.getVertices();
	/*for (int i = 0; i < vertices.size(); i++) {
		string st = "v" + to_string(i);
		Print_Vec(vertices[i], st);
	}*/
	m_pts.clear();
	for (int i = 0; i < vertices.size(); i++) {
		m_pts.push_back(vec(vertices[i].x, vertices[i].y, vertices[i].z));
	}
	m_KDTree = new KDTree(m_pts);
	temp_pts.clear();
	for (int i = 0; i < t_vertices.size(); i++) {
		temp_pts.push_back(vec(t_vertices[i].x, t_vertices[i].y, t_vertices[i].z));
	}
	temp_KDTree = new KDTree(temp_pts);
	//B3
	U1x = (float)GND_WIDTH / SEC_PER_FLOOR;
	U1 = vec(U1x / 3, 0, 0);
	U2 = vec(-U1x / 3, 0, 0);
	omega1 = vec(pi / 8, pi / 8, pi / 8);
	omega2 = vec(0, pi / 16, 0);
	collision_time = 0;
	initial_pos_1 = vec(-20, 0, 0);
	initial_pos_2 = vec(0, 0, 0);
	//B4
	e = 1.0;
}

void Mesh3DScene::resize()
{
	//! By Making `first_pass` static and initializing it to true,
	//! we make sure that the if block will be executed only once.
	static bool first_pass = true;
	if (first_pass)
	{
		printKeyboardShortcuts();
		m_model_original.setBigSize(getSceneWidth() / 2);
		m_model_original.move(vec(0, 0, 0));
		m_model_original.update();
		m_model = m_model_original;
		Find_CM(m_model.getVertices(), m_center_mass);
		m_center_mass += initial_pos_2;
		temp_B1_original.setBigSize(getSceneWidth() / 24);
		temp_B1_original.move(vec(0, 0, 0));
		temp_B1_original.update();
		temp_B1 = temp_B1_original;
		Find_CM(temp_B1.getVertices(), B1_CM);
		B1_CM += initial_pos_1;

		m_model_non_dup.setBigSize(getSceneWidth() / 2);
		m_model_non_dup.move(vec(0, 0, 0));
		m_model_non_dup.update();
		temp_B1_non_dup.setBigSize(getSceneWidth() / 24);
		temp_B1_non_dup.move(vec(0, 0, 0));
		temp_B1_non_dup.update();
		cout << temp_B1_non_dup.getVertices().size() << endl;

		//B4
		e = 1;
		dens1 = 0.1;
		dens2 = 0.1;
		find_Volume(temp_B1.getVertices(), temp_B1.getTriangles(), vol1);
		find_Volume(m_model.getVertices(), m_model.getTriangles(), vol2);
		find_Mass(vol1, dens1, m1);
		find_Mass(vol2, dens2, m2);
		cout << "mass1=" << m1 << ", mass2=" << m2 << endl;

		first_pass = false;
	}
}

void Mesh3DScene::arrowEvent(ArrowDir dir, int modif)
{
	math::vec n = m_plane.normal;
	if (dir == UP) m_plane_d += 1;
	if (dir == DOWN) m_plane_d -= 1;
	else if (dir == LEFT) n = math::float3x3::RotateY(DegToRad(1)).Transform(n);
	else if (dir == RIGHT) n = math::float3x3::RotateY(DegToRad(-1)).Transform(n);
	m_plane = Plane(n.Normalized(), m_plane_d);

}

void Mesh3DScene::keyEvent(unsigned char key, bool up, int modif)
{
	Scene::keyEvent(key, up, modif);
	key = tolower(key);

	switch (key)
	{
		FLAG_TOGGLE(m_style_flag, 'a', SHOW_AXES);
		FLAG_TOGGLE(m_style_flag, 's', SHOW_SOLID);
		FLAG_TOGGLE(m_style_flag, 'd', SHOW_WIRE);
		FLAG_TOGGLE(m_style_flag, 'n', SHOW_NORMALS);
		FLAG_TOGGLE(m_style_flag, 'p', SHOW_PLANE);
		FLAG_TOGGLE(m_style_flag, 'q', SHOW_TEMP_AABB);
		FLAG_TOGGLE(m_style_flag, 'w', SHOW_AABB);
		FLAG_TOGGLE(m_style_flag, 'e', SHOW_TEMP_SPHERE);
		FLAG_TOGGLE(m_style_flag, 't', SHOW_SPHERE);
		FLAG_TOGGLE(m_style_flag, '1', RUN_A1);
		FLAG_TOGGLE(m_style_flag, '2', RUN_A2);
		FLAG_TOGGLE(m_style_flag, '3', RUN_A3);
		FLAG_TOGGLE(m_style_flag, '4', RUN_B1);
		FLAG_TOGGLE(m_style_flag, '5', RUN_B2);
		FLAG_TOGGLE(m_style_flag, '6', RUN_B3);
		FLAG_TOGGLE(m_style_flag, '8', RUN_B4);
		FLAG_TOGGLE(m_style_flag, '!', SHOW_A1);
		FLAG_TOGGLE(m_style_flag, '@', SHOW_A2);
		FLAG_TOGGLE(m_style_flag, '#', SHOW_A3);
		FLAG_TOGGLE(m_style_flag, '$', SHOW_B1);
		FLAG_TOGGLE(m_style_flag, '%', SHOW_B2);
		FLAG_TOGGLE(m_style_flag, '^', SHOW_B3);
		FLAG_TOGGLE(m_style_flag, 'y', MOVE_B3);
		FLAG_TOGGLE(m_style_flag, '*', SHOW_B4);
		FLAG_TOGGLE(m_style_flag, 'g', TEMP_SOLID);
		FLAG_TOGGLE(m_style_flag, 'h', TEMP_WIRE);
		FLAG_TOGGLE(m_style_flag, 'l', LOAD_TEMP);
		FLAG_TOGGLE(m_style_flag, '0', RUN_B1_PLANE);
		FLAG_TOGGLE(m_style_flag, ')', SHOW_B1_PLANE);
	}

	if (key == ' ')
	{
		if (m_anim.paused()) m_anim.update(true); else m_anim.pause();
	}
	else if (key == '?')
	{
		printKeyboardShortcuts();
	}
	else if (key == 'u')
	{
		m_invalidation_sec = vvr::getSeconds();
	}
	else if (key == 'r') {
		printKeyboardShortcuts();
		m_model_original.setBigSize(getSceneWidth() / 2);
		m_model_original.update();
		m_model = m_model_original;
	}
}

void Mesh3DScene::printKeyboardShortcuts()
{
	std::cout << "Keyboard shortcuts:"
		<< std::endl << "'?' => This shortcut list:"
		<< std::endl << "'a' => SHOW_AXES"
		<< std::endl << "'s' => SHOW_SOLID"
		<< std::endl << "'d' => SHOW_WIRE"
		<< std::endl << "'r' => RESET_SCENE"
		<< std::endl << "'n' => SHOW_NORMALS"
		<< std::endl << "'p' => SHOW_PLANE"
		<< std::endl << "'q' => SHOW_TEMP_AABB"
		<< std::endl << "'w' => SHOW_AABB"
		<< std::endl << "'e' => SHOW_TEMP_SPHERE"
		<< std::endl << "'t' => SHOW_SPHERE"
		<< std::endl << "'1' => RUN_A1"
		<< std::endl << "'2' => RUN_A2"
		<< std::endl << "'3' => RUN_A3"
		<< std::endl << "'4' => RUN_B1"
		<< std::endl << "'5' => RUN_B2"
		<< std::endl << "'6' => RUN_B3"
		<< std::endl << "'8' => RUN_B4"
		<< std::endl << "'!' => SHOW_A1"
		<< std::endl << "'@' => SHOW_A2"
		<< std::endl << "'#' => SHOW_A3"
		<< std::endl << "'$' => SHOW_B1"
		<< std::endl << "'%' => SHOW_B2"
		<< std::endl << "'^' => SHOW_B3"
		<< std::endl << "'y' => MOVE_B3"
		<< std::endl << "'*' => SHOW_B4"
		<< std::endl << "'g' => TEMP_SOLID"
		<< std::endl << "'h' => TEMP_WIRE"
		<< std::endl << "'l' => LOAD_TEMP"
		<< std::endl << "'0' => RUN_B1_PLANE"
		<< std::endl << "')' => SHOW_B1_PLANE"
		<< std::endl << std::endl;
}

bool Mesh3DScene::idle()
{
	if (m_invalidation_sec > 0 &&
		vvr::getSeconds() - m_invalidation_sec > 0.8)
	{
		m_model = Mesh(objFile);
		m_model_original = m_model;
		temp_B1 = Mesh(cubeFile);
		temp_B1_original = temp_B1;
		m_invalidation_sec = -1;
	}
	m_anim.update();
	return true;
}

void Mesh3DScene::draw()
{
	m_last_update = vvr::getSeconds();



	//! Draw ground
	ground = Grounds(GND_WIDTH, GND_DEPTH, boundary.MinY() / 3, boundary.MaxY() / 3, vvr::Colour(35, 45, 55));
	//ground.draw();
	AABB& t_AABB = temp_B1_original.getAABB();
	AABB& m_AABB = m_model_original.getAABB();
	Sphere& t_sphere = temp_B1_original.getAABB().MaximalContainedSphere();
	Sphere& m_sphere = m_model_original.getAABB().MaximalContainedSphere();
	//! Animate mesh
	float t = m_anim.t;
	if (!m_anim.paused() || t == 0) {
		temp_B1 = Mesh(temp_B1_original);
		moveVertices(temp_B1.getVertices(), t, U1, omega1, final_transform1, temp_vertices, initial_pos_1);
		temp_B1.setTransform(final_transform1);
		temp_B1.update();

		moveVertices(m_model.getVertices(), t, U2, omega2, final_transform2, model_vertices, initial_pos_2);
		m_model.setTransform(final_transform2);
		m_model.update();
		Find_CM(temp_vertices, B1_CM);
		Find_CM(model_vertices, m_center_mass);

		math::float3x4 f_transf;
		moveVertices(m_model_non_dup.getVertices(), t, U2, omega2, f_transf, vertices_non_dup, initial_pos_2);
		m_model_non_dup.setTransform(f_transf);
		m_model_non_dup.update();

		//remake kdtrees
		if (FLAG_ON(m_style_flag, RUN_B1) || FLAG_ON(m_style_flag, RUN_B1_PLANE) || FLAG_ON(m_style_flag, RUN_B2) || FLAG_ON(m_style_flag, RUN_B3) || FLAG_ON(m_style_flag, SHOW_B1) || FLAG_ON(m_style_flag, SHOW_B1_PLANE) || FLAG_ON(m_style_flag, SHOW_B2) || FLAG_ON(m_style_flag, SHOW_B3)) {
			delete m_KDTree;
			delete temp_KDTree;
			m_pts.clear();
			for (int i = 0; i < model_vertices.size(); i++) {
				m_pts.push_back(vec(model_vertices[i].x, model_vertices[i].y, model_vertices[i].z));
			}
			m_KDTree = new KDTree(m_pts);
			temp_pts.clear();
			for (int i = 0; i < temp_vertices.size(); i++) {
				temp_pts.push_back(vec(temp_vertices[i].x, temp_vertices[i].y, temp_vertices[i].z));
			}
			temp_KDTree = new KDTree(temp_pts);
		}
	}
	findAABB(temp_vertices, t_AABB);
	findAABB(model_vertices, m_AABB);
	findSphere(temp_vertices, t_sphere);
	findSphere(model_vertices, m_sphere);
	Point3D(B1_CM.x, B1_CM.y, B1_CM.z, Colour::green).draw();
	Point3D(m_center_mass.x, m_center_mass.y, m_center_mass.z, Colour::red).draw();
	if ((abs(B1_CM.x) > 2 * GND_WIDTH) || (abs(m_center_mass.x) > 2 * GND_WIDTH)) {
		m_anim.setTime(0); // Bring back to start
	}

	aabb_vertices.clear();
	m_aabb_vertices.clear();
	for (int i = 0; i < 8; i++) {
		aabb_vertices.push_back(t_AABB.CornerPoint(i));
		m_aabb_vertices.push_back(m_AABB.CornerPoint(i));
		//cout << i << "=" << aabb_vertices[i] << ", ";
	}
	//cout << endl;

	if (FLAG_ON(m_style_flag, LOAD_TEMP)) {

		//Reset Meshes
		temp_A1 = Mesh();
		temp_A2 = Mesh();
		temp_A3 = Mesh();
		temp_B1 = Mesh();
		ground = Grounds();

		std::cout << "choose temp obj file:\n";
		int c = 1;
		for (int i = 0; i < obj_names.size(); i++) {
			std::cout << "(" << c << ")" << obj_names[i] << ", ";
			c++;
		}
		std::cout << endl;
		int objNo;
		cin >> objNo;
		std::cout << "Current temp Object:" << obj_names[objNo - 1] << endl;
		cubeFile = file_paths[objNo - 1];
		temp_B1_original = Mesh(cubeFile);
		temp_B1_original.setBigSize(getSceneWidth() / 24);
		temp_B1_original.move(vec(0, 0, 0));
		temp_B1_original.update();
		temp_B1 = temp_B1_original;
		Find_CM(temp_B1.getVertices(), B1_CM);

		m_anim.setTime(0);
		t = 0;
		delete m_KDTree;
		delete temp_KDTree;
		vector<vec>& vertices = m_model_original.getVertices();
		vector<vec>& t_vertices = temp_B1_original.getVertices();
		m_pts.clear();
		for (int i = 0; i < vertices.size(); i++) {
			m_pts.push_back(vec(vertices[i].x, vertices[i].y, vertices[i].z));
		}
		m_KDTree = new KDTree(m_pts);
		temp_pts.clear();
		for (int i = 0; i < t_vertices.size(); i++) {
			temp_pts.push_back(vec(t_vertices[i].x, t_vertices[i].y, t_vertices[i].z));
		}
		temp_KDTree = new KDTree(temp_pts);

		signed_distance_function_temp.clear();
		signed_distance_function.clear();
		normals.clear();
		normals_temp.clear();

		float U1x = (float)GND_WIDTH / SEC_PER_FLOOR;
		U1 = vec(U1x / 3, 0, 0);
		U2 = vec(-U1x / 3, 0, 0);
		omega1 = vec(pi / 8, pi / 8, pi / 8);
		omega2 = vec(0, pi / 16, 0);
		collision_time = 0;
		initial_pos_1 = vec(-20, 0, 0);
		initial_pos_2 = vec(0, 0, 0);

		e = 1;
		dens1 = 0.1;
		dens2 = 0.1;
		find_Volume(temp_B1.getVertices(), temp_B1.getTriangles(), vol1);
		find_Volume(m_model.getVertices(), m_model.getTriangles(), vol2);
		find_Mass(vol1, dens1, m1);
		find_Mass(vol2, dens2, m2);
		cout << "mass1=" << m1 << ", mass2=" << m2 << endl;

		m_style_flag -= FLAG(LOAD_TEMP);
	}

	//A_1
	if (FLAG_ON(m_style_flag, SHOW_A1)) {
		//temp_A1.draw(vvr::Colour::red, SOLID);
		vector<vec>& vert = temp_A1.getVertices();
		//temp_A1.draw(vvr::Colour::black, WIRE);
		for (int i = 0; i < Dist.size(); i++) {
			Colour c = Colour(qDist[i], 255 - qDist[i], 0);
			//std::cout << qDist[i] << endl;
			vvr::Point3D dp = Point3D(vert[Dist[i].index].x, vert[Dist[i].index].y, vert[Dist[i].index].z, c);
			dp.colour = c;
			dp.draw();
			vvr::LineSeg3D line = LineSeg3D(dp.x, dp.y, dp.z, dp.x * vert[Dist[i].index].Length(), dp.y * vert[Dist[i].index].Length(), dp.z * vert[Dist[i].index].Length(), c);
			line.draw();
		}
	}
	if (FLAG_ON(m_style_flag, RUN_A1)) {
		//Prepare Matrixes and other variables
		vector<vvr::Triangle>& m_triangles = m_model.getTriangles();
		vector<vec>& m_vertices = m_model.getVertices();
		int verticesCount = m_vertices.size();
		MatrixXf Coords(m_model.getVertices().size(), 3);
		SparseMatrix<float> L(verticesCount, verticesCount);
		MatrixXf DifCoords(verticesCount, 3);
		MatrixXf DifCoords_Check(verticesCount, 3);
		float find_dur, lambda, me, task_dur;
		int N;
		for (int i = 0; i < m_model.getVertices().size(); i++) {
			Coords(i, 0) = m_model.getVertices()[i].x;
			Coords(i, 1) = m_model.getVertices()[i].y;
			Coords(i, 2) = m_model.getVertices()[i].z;
		}
		//find L matrix
		find_dur = vvr::getSeconds();
		find_L_Matrix(m_triangles, L, Coords);
		find_dur = vvr::getSeconds() - find_dur;
		std::cout << "found L matrix in time=" << find_dur << endl << endl;

		temp_A1 = vvr::Mesh(objFile);
		temp_A1.setBigSize(getSceneWidth() / 2);
		task_dur = vvr::getSeconds();
		Task_A_1_Find_Differential(L, Coords, DifCoords);
		ReplaceVertices(temp_A1.getVertices(), DifCoords);
		temp_A1.update();

		//show Diffs
		Dist.clear();
		qDist.clear();
		for (int i = 0; i < verticesCount; i++) {
			diffDistance d;
			d.dist = sqrt(pow(DifCoords(i, 0), 2) + pow(DifCoords(i, 1), 2) + pow(DifCoords(i, 2), 2));
			d.index = i;
			Dist.push_back(d);
		}
		sort(Dist.begin(), Dist.end(), Distcomparator);
		qDist.push_back(0);
		qDist.insert(qDist.end(), 255);
		float b = Dist[Dist.size() - 1].dist - Dist[0].dist;
		for (int i = 1; i < verticesCount - 1; i++) {
			float p = Dist[i].dist / b;
			int n = int(p * 255);
			qDist.push_back(n);
		}

		task_dur = vvr::getSeconds() - task_dur;
		std::cout << "task time=" << task_dur << endl;
		m_style_flag -= FLAG(RUN_A1);
	}

	//A_2
	if (FLAG_ON(m_style_flag, SHOW_A2)) {
		temp_A2.draw(vvr::Colour::red, SOLID);
		//temp_A2.draw(vvr::Colour::black, WIRE);
	}
	if (FLAG_ON(m_style_flag, RUN_A2)) {
		temp_A2 = vvr::Mesh(objFile);
		temp_A2.setBigSize(getSceneWidth() / 2);
		//Prepare Matrixes and other variables
		vector<vvr::Triangle>& m_triangles = m_model.getTriangles();
		vector<vec>& m_vertices = m_model.getVertices();
		int verticesCount = m_vertices.size();
		MatrixXf Coords(m_model.getVertices().size(), 3);
		SparseMatrix<float> L(verticesCount, verticesCount);
		MatrixXf DifCoords(verticesCount, 3);
		MatrixXf DifCoords_Check(verticesCount, 3);
		float find_dur, lambda, me, task_dur;
		int N;
		for (int i = 0; i < m_model.getVertices().size(); i++) {
			Coords(i, 0) = m_model.getVertices()[i].x;
			Coords(i, 1) = m_model.getVertices()[i].y;
			Coords(i, 2) = m_model.getVertices()[i].z;
		}
		//find L matrix
		find_dur = vvr::getSeconds();
		find_L_Matrix(m_triangles, L, Coords);
		find_dur = vvr::getSeconds() - find_dur;
		std::cout << "found L matrix in time=" << find_dur << endl << endl;
		//m_style_flag &= !FLAG(RUN_PREP);

		//N = 100;
		//lambda = 0.5;
		std::cout << "press -1 anytime to disable this task\n";
		std::cout << "give number of iterations:";
		cin >> N;
		if (N == -1) {
			m_style_flag &= !FLAG(RUN_A3);
			std::cout << endl;
		}
		else {
			std::cout << N << endl;
			std::cout << "give float number lambda:";
			cin >> lambda;
			if (lambda == -1) {
				m_style_flag &= !FLAG(RUN_A3);
				std::cout << endl;
			}
			else {
				std::cout << lambda << endl;
				task_dur = vvr::getSeconds();
				Task_A2_Smoothing(L, Coords, DifCoords, N, lambda);
				ReplaceVertices(temp_A2.getVertices(), DifCoords);
				temp_A2.update();
				task_dur = vvr::getSeconds() - task_dur;
				std::cout << "task time=" << task_dur << endl;
			}
		}
		m_style_flag -= FLAG(RUN_A2);
	}

	//A_3
	if (FLAG_ON(m_style_flag, SHOW_A3)) {
		temp_A3.draw(m_obj_col, SOLID);
		temp_A3.draw(vvr::Colour::black, WIRE);
	}
	if (FLAG_ON(m_style_flag, RUN_A3)) {
		temp_A3 = vvr::Mesh(objFile);
		temp_A3.setBigSize(getSceneWidth() / 2);

		//Prepare Matrixes and other variables
		vector<vvr::Triangle>& m_triangles = m_model.getTriangles();
		vector<vec>& m_vertices = m_model.getVertices();
		int verticesCount = m_vertices.size();
		MatrixXf Coords(m_model.getVertices().size(), 3);
		SparseMatrix<float> L(verticesCount, verticesCount);
		MatrixXf DifCoords(verticesCount, 3);
		MatrixXf DifCoords_Check(verticesCount, 3);
		float find_dur, lambda, me, task_dur;
		int N;
		for (int i = 0; i < m_model.getVertices().size(); i++) {
			Coords(i, 0) = m_model.getVertices()[i].x;
			Coords(i, 1) = m_model.getVertices()[i].y;
			Coords(i, 2) = m_model.getVertices()[i].z;
		}
		//find L matrix
		find_dur = vvr::getSeconds();
		find_L_Matrix(m_triangles, L, Coords);
		find_dur = vvr::getSeconds() - find_dur;
		std::cout << "found L matrix in time=" << find_dur << endl << endl;
		//m_style_flag &= !FLAG(RUN_PREP);

		//Task A_3
		//N = 100;
		std::cout << "press -1 anytime to disable this task\n";
		std::cout << "give number of iterations:";
		cin >> N;
		if (N == -1) {
			m_style_flag &= !FLAG(RUN_A3);
			std::cout << endl;
		}
		else {
			std::cout << N << endl;
			std::cout << "give float number lambda:";
			cin >> lambda;
			if (lambda == -1) {
				m_style_flag &= !FLAG(RUN_A3);
				std::cout << endl;
			}
			else {
				std::cout << lambda << endl;
				std::cout << "give (positive) float number me:";
				cin >> me;
				if (me == -1) {
					m_style_flag &= !FLAG(RUN_A3);
					std::cout << endl;
				}
				else {
					std::cout << me << endl;
					task_dur = vvr::getSeconds();
					Task_A3_Taubin_Smoothing(L, Coords, DifCoords, N, lambda, me);
					ReplaceVertices(temp_A3.getVertices(), DifCoords);
					temp_A3.update();
					task_dur = vvr::getSeconds() - task_dur;
					std::cout << "task time=" << task_dur << endl;
				}
			}
		}
		m_style_flag -= FLAG(RUN_A3);
	}


	//B_1
	if (FLAG_ON(m_style_flag, SHOW_B1)) {
		for (int i = 0; i < signed_distance_function.size(); i++) {
			signed_distance sd = signed_distance_function[i];
			if (sd.d > 0) {
				//LineSeg3D(sd.p.x, sd.p.y-20, sd.p.z, sd.p.x, sd.p.y - 20 + sd.d / 5.0, sd.p.z, Colour::orange).draw();
				Point3D(sd.p.x, -30 + sd.d, sd.p.z, Colour::orange).draw();
			}
			else {
				//LineSeg3D(sd.p.x, sd.p.y-20, sd.p.z, sd.p.x, sd.p.y - 20 + sd.d / 5.0, sd.p.z, Colour::darkRed).draw();
				Point3D(sd.p.x, -30 + sd.d, sd.p.z, Colour::darkRed).draw();
			}
			AABB ab = m_model.getAABB();
			float minx = ab.MinX();
			float minz = ab.MinZ();
			float maxx = ab.MaxX();
			float maxz = ab.MaxZ();
			vvr::Colour colPlane(0x41, 0x14, 0xB3);
			vec p0 = vec(minx, -30, minz);
			vec p1 = vec(minx, -30, maxz);
			vec p2 = vec(maxx, -30, minz);
			vec p3 = vec(maxx, -30, maxz);
			math2vvr(math::Triangle(p0, p1, p2), colPlane).draw();
			math2vvr(math::Triangle(p2, p1, p3), colPlane).draw();
		}
	}
	if (FLAG_ON(m_style_flag, SHOW_B1_PLANE)) {
		AABB ab = m_model.getAABB();

		float minx = ab.MinX();
		float minz = ab.MinZ();
		float maxx = ab.MaxX();
		float maxz = ab.MaxZ();
		vvr::Colour colPlane(0x41, 0x14, 0xB3);
		vec p0 = vec(minx, y, minz);
		vec p1 = vec(minx, y, maxz);
		vec p2 = vec(maxx, y, minz);
		vec p3 = vec(maxx, y, maxz);
		math2vvr(math::Triangle(p0, p1, p2), colPlane).draw();
		math2vvr(math::Triangle(p2, p1, p3), colPlane).draw();
	}
	if (FLAG_ON(m_style_flag, RUN_B1)) {
		static bool first_time = true;
		if (first_time) {
			cout << "choose mode to find sdf: '1'=sphere , '2'=AABB , '3'=object , '4'=test , '5'=siren" << endl;
			cin >> option_B1;
			first_time = false;
		}

		signed_distance_function.clear();
		if (obj_names[tempNo - 1] == "cube") {
			float t1 = vvr::getSeconds();
			findSigned_Cube(temp_vertices, temp_B1.getTriangles(), final_transform1, model_vertices, signed_distance_function, m_model.getTriangles());
			t1 = vvr::getSeconds() - t1;
			cout << "Time for cube function = " << t1 << endl;
		}
		else {
			float t1;
			string script_path = PROJECT_ROOT + "/test/siren/sdf_model_create.py";
			//string script_path = PROJECT_ROOT + "/test/siren/dist/sdf_model_create.exe";
			switch (option_B1) {
			case(1):
				t1 = vvr::getSeconds();
				findSigned_Sphere(t_sphere, model_vertices, signed_distance_function);
				t1 = vvr::getSeconds() - t1;
				cout << "Time for sphere function = " << t1 << endl;
				break;
			case(2):
				t1 = vvr::getSeconds();
				findSigned_AABB(t_AABB, aabb_vertices, model_vertices, signed_distance_function);
				t1 = vvr::getSeconds() - t1;
				cout << "Time for aabb function = " << t1 << endl;
				break;
			case(3):
				t1 = vvr::getSeconds();
				findSigned_KDTree(temp_KDTree, model_vertices, temp_vertices, signed_distance_function, temp_B1.getTriangles());
				t1 = vvr::getSeconds() - t1;
				cout << "Time for kdtree function = " << t1 << endl;
				break;
			case(4):
				t1 = vvr::getSeconds();
				for (int i = 0; i < 1000; i++) {
					signed_distance_function.clear();
					findSigned_AABB(t_AABB, aabb_vertices, model_vertices, signed_distance_function);
					//findSigned_Cube( temp_vertices,temp_B1.getTriangles(),final_transform1, model_vertices, signed_distance_function);
					//findSigned_Brute( temp_vertices, model_vertices, signed_distance_function, m_model.getTriangles());
				}
				t1 = vvr::getSeconds() - t1;
				t1 /= 1000;
				cout << "Average time for aabb function = " << t1 << endl;

				t1 = vvr::getSeconds();
				for (int i = 0; i < 10; i++) {
					signed_distance_function.clear();
					findSigned_KDTree(temp_KDTree, model_vertices, temp_vertices, signed_distance_function, temp_B1.getTriangles());
					//findSigned_KDTree(m_KDTree, temp_vertices, model_vertices, signed_distance_function, m_model.getTriangles());
				}
				t1 = vvr::getSeconds() - t1;
				t1 /= 10;
				cout << "Average time for kd-tree function = " << t1 << endl;
				FLAG_OFF(m_style_flag, RUN_B1);
				break;
			case(5):
				t1 = vvr::getSeconds();
				savePointsToText(temp_vertices);
				system(script_path.c_str());
				getSdfFromText(signed_distance_function, temp_vertices);
				t1 = vvr::getSeconds() - t1;
				cout << "Time for siren function = " << t1 << endl;
				sort(signed_distance_function.begin(), signed_distance_function.end(), sdf_comp);
				cout << "min=" << signed_distance_function[0].d << " at (" << signed_distance_function[0].p.x << "," << signed_distance_function[0].p.y << "," << signed_distance_function[0].p.z << ")" << endl;
				break;
			default:
				//findSigned_Brute(temp_vertices, model_vertices, signed_distance_function, m_model.getTriangles());
				t1 = vvr::getSeconds();
				findSigned_KDTree(m_KDTree, temp_vertices, model_vertices, signed_distance_function, m_model.getTriangles());
				t1 = vvr::getSeconds() - t1;
				cout << "Time for B_1) function = " << t1 << endl;
			}
		}
		FLAG_OFF(m_style_flag, RUN_B1);
	}

	if (FLAG_ON(m_style_flag, RUN_B1_PLANE)) {
		cout << "choose mode to find sdf: '1'=sphere , '2'=AABB , '3'=siren , '4'=object" << endl;
		cin >> option_B1;
		vector<vec> plane_points;
		AABB ab = m_model.getAABB();
		y = -20;
		std::cout << "choose height (from -20 to 20)" << endl;
		cin >> y;
		float dx = ab.MaxX() - ab.MinX();
		float dz = ab.MaxZ() - ab.MinZ();
		float minx = ab.MinX();
		float minz = ab.MinZ();
		int countx = int(dx * 2);
		int countz = int(dz * 2);
		std::cout << countx << "," << countz << endl;
		for (int i = 0; i < countx; i++) {
			for (int j = 0; j < countz; j++) {
				plane_points.push_back(vec(minx + 0.5 * i, y, minz + 0.5 * j));
			}
		}
		std::cout << "end" << endl;
		/*
		float t = vvr::getSeconds();
		for (int i = 0; i < 20; i++) {
			signed_distance_function.clear();
			findSigned_Brute(plane_points, model_vertices, signed_distance_function, m_model.getTriangles());
		}
		t = vvr::getSeconds() - t;
		t /= 20;
		cout << "Average time for brute force function = "<<t<<endl;

		t = vvr::getSeconds();
		for (int i = 0; i < 20; i++) {
			signed_distance_function.clear();
			findSigned_KDTree(m_KDTree,plane_points, model_vertices, signed_distance_function, m_model.getTriangles());
		}
		t = vvr::getSeconds() - t;
		t /= 20;
		cout << "Average time for kd-tree function = " << t << endl;
		*/
		signed_distance_function.clear();
		string script_path = PROJECT_ROOT + "/test/siren/sdf_model_create.py";
		//string script_path = PROJECT_ROOT + "/test/siren/dist/sdf_model_create.exe";
		if (obj_names[objNo - 1] == "cube") {
			vector<vvr::Triangle> tr_empty;
			findSigned_Cube(model_vertices, m_model.getTriangles(), final_transform2, plane_points, signed_distance_function, tr_empty);
		}
		else {
			if (option_B1 == 1) {
				findSigned_Sphere(m_sphere, plane_points, signed_distance_function);
			}
			else if (option_B1 == 2) {
				findSigned_AABB(m_AABB, m_aabb_vertices, plane_points, signed_distance_function);
			}
			else if (option_B1 == 3) {
				float t1 = vvr::getSeconds();
				savePointsToText(plane_points);
				system(script_path.c_str());
				getSdfFromText(signed_distance_function, plane_points);
				t1 = vvr::getSeconds() - t1;
				cout << "Time for neural function = " << t1 << endl;
				sort(signed_distance_function.begin(), signed_distance_function.end(), sdf_comp);
				cout << "min=" << signed_distance_function[0].d << " at (" << signed_distance_function[0].p.x << "," << signed_distance_function[0].p.y << "," << signed_distance_function[0].p.z << ")" << endl;

			}
			else {
				float t1 = vvr::getSeconds();
				findSigned_KDTree(m_KDTree, plane_points, model_vertices, signed_distance_function, m_model.getTriangles());
				t1 = vvr::getSeconds() - t1;
				cout << "Time for B_1) function = " << t1 << endl;
			}
		}
		FLAG_OFF(m_style_flag, RUN_B1);
		FLAG_OFF(m_style_flag, RUN_B1_PLANE);
		m_style_flag |= FLAG(SHOW_B1_PLANE);
	}
	if (FLAG_ON(m_style_flag, SHOW_B2)) {
		if (obj_names[objNo - 1] == "cube") {
			for (int i = 0; i < normals.size(); i++) {
				vec x = vec(vertices_non_dup[normals[i].v].x, vertices_non_dup[normals[i].v].y, vertices_non_dup[normals[i].v].z);
				vec x_n = x - (normals[i].n / normals[i].n.Length());
				LineSeg3D(x.x, x.y, x.z, x_n.x, x_n.y, x_n.z, Colour::red).draw();
			}
		}
		else {
			for (int i = 0; i < normals.size(); i++) {
				//show normals of temp_model
				//vec x = vec(temp_vertices[normals[i].v].x, temp_vertices[normals[i].v].y, temp_vertices[normals[i].v].z);
				//show normals of m_model
				vec x = vec(model_vertices[normals[i].v].x, model_vertices[normals[i].v].y, model_vertices[normals[i].v].z);
				vec x_n = x + (normals[i].n / normals[i].n.Length());
				LineSeg3D(x.x, x.y, x.z, x_n.x, x_n.y, x_n.z, Colour::red).draw();
			}

			/*
			vec n;
			vec x = vec(model_vertices[1700].x, model_vertices[1700].y, model_vertices[1700].z);
			vector<int> tr_inds;
			find_Normal_Point(x, n, model_vertices, m_model.getTriangles(),tr_inds);
			vec x_n = x - n;
			cout << "n=(" << n.x/n.Length() << "," << n.y / n.Length() << "," << n.z / n.Length() <<")"<<", L="<<n.Length() << endl;
			LineSeg3D(x.x, x.y, x.z, x_n.x, x_n.y, x_n.z, Colour::black).draw();
			for (int j = 0; j < tr_inds.size(); j++) {
				vvr::Triangle& t = m_model.getTriangles()[tr_inds[j]];
				vec p0 = vec(model_vertices[t.v[0]].x, model_vertices[t.v[0]].y, model_vertices[t.v[0]].z);
				vec p1 = vec(model_vertices[t.v[1]].x, model_vertices[t.v[1]].y, model_vertices[t.v[1]].z);
				vec p2 = vec(model_vertices[t.v[2]].x, model_vertices[t.v[2]].y, model_vertices[t.v[2]].z);
				math2vvr(math::Triangle(p0, p1, p2), Colour::green).draw();
			}
			*/
		}
	}

	if (FLAG_ON(m_style_flag, RUN_B2)) {
		//find normals of temp_model only for negative DF
		signed_distance_function.clear();
		normals.clear();
		//findSigned_KDTree(m_KDTree, temp_vertices,model_vertices, signed_distance_function, m_model.getTriangles());
		//find_Normals(normals, temp_vertices, signed_distance_function, temp_B1.getTriangles());
		if (obj_names[objNo - 1] == "cube") {
			find_Normals_all(normals, vertices_non_dup, m_model_non_dup.getTriangles());
		}
		else {
			find_Normals_all(normals, model_vertices, m_model.getTriangles());
			//findSigned_KDTree(temp_KDTree, model_vertices, temp_vertices, signed_distance_function, temp_B1.getTriangles());
			//find_Normals(normals, model_vertices, signed_distance_function, m_model.getTriangles());

		}
		//find normals of m_model only for negative DF
		//findSigned_KDTree(temp_KDTree, model_vertices, temp_vertices, signed_distance_function, temp_B1.getTriangles());
		//find_Normals(normals, model_vertices, signed_distance_function, m_model.getTriangles());
		m_style_flag -= FLAG(RUN_B2);
	}
	if (FLAG_ON(m_style_flag, SHOW_B3)) {

		//Point3D(contact_p.x, contact_p.y, contact_p.z, Colour::magenta).draw();
		vec x_n = contact_p + (contact_normal / contact_normal.Length());
		//LineSeg3D(contact_p.x, contact_p.y, contact_p.z, x_n.x, x_n.y, x_n.z, Colour::darkGreen).draw();


		//Point3D(contact_p_th.x, contact_p_th.y, contact_p_th.z, Colour::darkRed).draw();
		vec x_n_2 = contact_p_th + (contact_normal_th / contact_normal_th.Length());
		//LineSeg3D(contact_p_th.x, contact_p_th.y, contact_p_th.z, x_n_2.x, x_n_2.y, x_n_2.z, Colour::yellowGreen).draw();

	}

	if (FLAG_ON(m_style_flag, RUN_B3)) {
		//reset positions
		m_anim.setTime(0);
		t = 0;
		U1 = vec(U1x / 3, 0, 0);
		U2 = vec(-U1x / 3, 0, 0);
		omega1 = vec(pi / 8, pi / 8, pi / 8);
		omega2 = vec(0, pi / 16, 0);
		collision_time = 0;
		initial_pos_1 = vec(-20, 0, 0);
		initial_pos_2 = vec(0, 0, 0);

		temp_B1 = Mesh(temp_B1_original);
		moveVertices(temp_B1.getVertices(), t, U1, omega1, final_transform1, temp_vertices, initial_pos_1);
		temp_B1.setTransform(final_transform1);
		temp_B1.update();

		moveVertices(m_model.getVertices(), t, U2, omega2, final_transform2, model_vertices, initial_pos_2);
		m_model.setTransform(final_transform2);
		m_model.update();
		Find_CM(temp_vertices, B1_CM);
		Find_CM(model_vertices, m_center_mass);

		math::float3x4 f_transf;
		moveVertices(m_model_non_dup.getVertices(), t, U2, omega2, f_transf, vertices_non_dup, initial_pos_2);
		m_model_non_dup.setTransform(f_transf);
		m_model_non_dup.update();

		//remake kdtrees
		if (FLAG_ON(m_style_flag, RUN_B1) || FLAG_ON(m_style_flag, RUN_B1_PLANE) || FLAG_ON(m_style_flag, RUN_B2) || FLAG_ON(m_style_flag, RUN_B3) || FLAG_ON(m_style_flag, SHOW_B1) || FLAG_ON(m_style_flag, SHOW_B1_PLANE) || FLAG_ON(m_style_flag, SHOW_B2) || FLAG_ON(m_style_flag, SHOW_B3)) {
			delete m_KDTree;
			delete temp_KDTree;
			m_pts.clear();
			for (int i = 0; i < model_vertices.size(); i++) {
				m_pts.push_back(vec(model_vertices[i].x, model_vertices[i].y, model_vertices[i].z));
			}
			m_KDTree = new KDTree(m_pts);
			temp_pts.clear();
			for (int i = 0; i < temp_vertices.size(); i++) {
				temp_pts.push_back(vec(temp_vertices[i].x, temp_vertices[i].y, temp_vertices[i].z));
			}
			temp_KDTree = new KDTree(temp_pts);
		}
		findAABB(temp_vertices, t_AABB);
		findSphere(temp_vertices, t_sphere);
		aabb_vertices.clear();
		for (int i = 0; i < 8; i++) {
			aabb_vertices.push_back(t_AABB.CornerPoint(i));
		}

		cout << "choose mode to find collision:'1'=sphere , '2'=AABB , '3'=siren , '4'=object" << endl;
		cin >> option_B3;
		cout << "use sdf? (y/n)" << endl;
		string s;
		cin >> s;
		string script_path = PROJECT_ROOT + "/test/siren/sdf_model_create.py";
		//string script_path = PROJECT_ROOT + "/test/siren/dist/sdf_model_create.exe";
		if (s == "y") use_sdf = true;
		else use_sdf = false;
		if (use_sdf) {
			if (obj_names[tempNo - 1] == "cube") {
				cout << "running cube with mesh collision detection" << endl;
				float sec = vvr::getSeconds();
				signed_distance_function_temp.clear();
				findSigned_Cube(temp_vertices, temp_B1.getTriangles(), final_transform1, model_vertices, signed_distance_function_temp, m_model.getTriangles());
				//calc collision step
				bool cont = true;
				float step = 0.1;
				float next_t = t;
				sort(signed_distance_function_temp.begin(), signed_distance_function_temp.end(), sdf_comp);
				float sdf_min_temp = signed_distance_function_temp[0].d;
				while (cont) {
					vec dl1 = (U1 - U2) * next_t;
					//cout << dl1.x << endl;
					if (sdf_min_temp - dl1.Length() <= 0) {
						cont = false;
						//converge to collision point
						collision_time = next_t;
						cout << "initial coll_time=" << collision_time << endl;
						Converge_Collision_Cube(temp_B1, m_model.getVertices(), signed_distance_function_temp, m_model.getTriangles(), collision_time, U1, U2, omega1, omega2, initial_pos_1, initial_pos_2, contact_p_th, contact_index, contact_normal_th);
						cout << "final coll_time=" << collision_time << endl;
					}
					if (cont) {
						next_t += step;
					}

				}
				sec = vvr::getSeconds() - sec;
				cout << "Found collision time in " << sec << " seconds." << endl;
			}
			else if (option_B3 == 1) {
				cout << "running sphere with mesh collision detection" << endl;
				float sec = vvr::getSeconds();
				signed_distance_function_temp.clear();
				findSigned_Sphere(t_sphere, model_vertices, signed_distance_function_temp);
				//calc collision step
				bool cont = true;
				float step = 0.1;
				float next_t = t;
				sort(signed_distance_function_temp.begin(), signed_distance_function_temp.end(), sdf_comp);
				float sdf_min_temp = signed_distance_function_temp[0].d;
				while (cont) {
					vec dl1 = (U1 - U2) * next_t;
					//cout << dl1.x << endl;
					if (sdf_min_temp - dl1.Length() <= 0) {
						cont = false;
						//converge to collision point
						collision_time = next_t;
						cout << "initial coll_time=" << collision_time << endl;
						Converge_Collision_Sphere(temp_B1, temp_B1.getVertices(), m_model.getVertices(), signed_distance_function_temp, m_model.getTriangles(), collision_time, U1, U2, omega1, omega2, initial_pos_1, initial_pos_2, contact_p_th, contact_index, contact_normal_th);
						cout << "final coll_time=" << collision_time << endl;
					}
					if (cont) {
						next_t += step;
					}

				}
				sec = vvr::getSeconds() - sec;
				cout << "Found collision time in " << sec << " seconds." << endl;
			}
			else if (option_B3 == 2) {
				cout << "running aabb with mesh collision detection" << endl;
				float sec = vvr::getSeconds();
				signed_distance_function_temp.clear();
				findSigned_AABB(t_AABB, aabb_vertices, model_vertices, signed_distance_function_temp);
				//calc collision step
				bool cont = true;
				float step = 0.1;
				float next_t = t;
				sort(signed_distance_function_temp.begin(), signed_distance_function_temp.end(), sdf_comp);
				float sdf_min_temp = signed_distance_function_temp[0].d;
				while (cont) {
					vec dl1 = (U1 - U2) * next_t;
					//cout << dl1.x << endl;
					if (sdf_min_temp - dl1.Length() <= 0) {
						cont = false;
						//converge to collision point
						collision_time = next_t;
						cout << "initial coll_time=" << collision_time << endl;
						Converge_Collision_AABB(temp_B1, temp_B1.getVertices(), m_model.getVertices(), signed_distance_function_temp, m_model.getTriangles(), collision_time, U1, U2, omega1, omega2, initial_pos_1, initial_pos_2, contact_p_th, contact_index, contact_normal_th);
						cout << "final coll_time=" << collision_time << endl;
					}
					if (cont) {
						next_t += step;
					}

				}
				sec = vvr::getSeconds() - sec;
				cout << "Found collision time in " << sec << " seconds." << endl;
			}
			else if (option_B3 == 3) {
				cout << "running collision detection with siren" << endl;
				float sec = vvr::getSeconds();
				signed_distance_function_temp.clear();
				savePointsToText(temp_vertices);
				system(script_path.c_str());
				getSdfFromText(signed_distance_function_temp, temp_vertices);
				//calc collision step
				bool cont = true;
				float step = 0.1;
				float next_t = t;
				sort(signed_distance_function_temp.begin(), signed_distance_function_temp.end(), sdf_comp);
				float sdf_min_temp = signed_distance_function_temp[0].d;
				while (cont) {
					vec dl1 = (U1 - U2) * next_t;
					//cout << dl1.x << endl;
					if (sdf_min_temp - dl1.Length() <= 0) {
						cont = false;
						//converge to collision point
						collision_time = next_t;
						cout << "initial coll_time=" << collision_time << endl;
						Converge_Collision_SIREN(temp_B1.getVertices(), m_model.getVertices(), signed_distance_function_temp, temp_B1.getTriangles(), m_model.getTriangles(), collision_time, U1, U2, omega1, omega2, initial_pos_1, initial_pos_2, contact_p_th, contact_index, contact_normal_th);
						cout << "final coll_time=" << collision_time << endl;
					}
					if (cont) {
						next_t += step;
					}

				}
				sec = vvr::getSeconds() - sec;
				cout << "Found collision time in " << sec << " seconds." << endl;
			}
			else {
				cout << "running collision detection" << endl;
				float sec = vvr::getSeconds();
				signed_distance_function_temp.clear();
				findSigned_KDTree(m_KDTree, temp_vertices, model_vertices, signed_distance_function_temp, m_model.getTriangles());
				//calc collision step
				bool cont = true;
				float step = 0.1;
				float next_t = t;
				sort(signed_distance_function_temp.begin(), signed_distance_function_temp.end(), sdf_comp);
				float sdf_min_temp = signed_distance_function_temp[0].d;
				while (cont) {
					vec dl1 = (U1 - U2) * next_t;
					//cout << dl1.x << endl;
					if (sdf_min_temp - dl1.Length() <= 0) {
						cont = false;
						//converge to collision point
						collision_time = next_t;
						cout << "initial coll_time=" << collision_time << endl;
						Converge_Collision(temp_B1.getVertices(), m_model.getVertices(), signed_distance_function_temp, temp_B1.getTriangles(), m_model.getTriangles(), collision_time, U1, U2, omega1, omega2, initial_pos_1, initial_pos_2, contact_p_th, contact_index, contact_normal_th);
						cout << "final coll_time=" << collision_time << endl;
					}
					if (cont) {
						next_t += step;
					}

				}
				sec = vvr::getSeconds() - sec;
				cout << "Found collision time in " << sec << " seconds." << endl;
			}
		}
		else {
			if (obj_names[tempNo - 1] == "cube") {
				cout << "running cube with mesh collision detection without sdf" << endl;
				float sec = vvr::getSeconds();
				collision_time = 0;
				cout << "initial coll_time=" << collision_time << endl;
				Converge_Collision_Cube_Nosdf(temp_B1, m_model.getVertices(), m_model.getTriangles(), collision_time, U1, U2, omega1, omega2, initial_pos_1, initial_pos_2, contact_p_th, contact_index, contact_normal_th);
				cout << "final coll_time=" << collision_time << endl;
				sec = vvr::getSeconds() - sec;
				cout << "Found collision time in " << sec << " seconds." << endl;
			}
			else if (option_B3 == 1) {
				cout << "running sphere with mesh collision detection without sdf" << endl;
				float sec = vvr::getSeconds();
				collision_time = 0;
				cout << "initial coll_time=" << collision_time << endl;
				Converge_Collision_Sphere_Nosdf(temp_B1, temp_B1.getVertices(), m_model.getVertices(), m_model.getTriangles(), collision_time, U1, U2, omega1, omega2, initial_pos_1, initial_pos_2, contact_p_th, contact_index, contact_normal_th);
				cout << "final coll_time=" << collision_time << endl;
				sec = vvr::getSeconds() - sec;
				cout << "Found collision time in " << sec << " seconds." << endl;
			}
			else if (option_B3 == 2) {
				cout << "running aabb with mesh collision detection without sdf" << endl;
				float sec = vvr::getSeconds();
				collision_time = 0;
				cout << "initial coll_time=" << collision_time << endl;
				Converge_Collision_AABB_Nosdf(temp_B1, temp_B1.getVertices(), m_model.getVertices(), m_model.getTriangles(), collision_time, U1, U2, omega1, omega2, initial_pos_1, initial_pos_2, contact_p_th, contact_index, contact_normal_th);
				cout << "final coll_time=" << collision_time << endl;
				sec = vvr::getSeconds() - sec;
				cout << "Found collision time in " << sec << " seconds." << endl;
			}
			else {
				cout << "running collision detection without sdf" << endl;
				float sec = vvr::getSeconds();
				collision_time = 0;
				cout << "initial coll_time=" << collision_time << endl;
				Converge_Collision_Nosdf(temp_B1.getVertices(), m_model.getVertices(), temp_B1.getTriangles(), m_model.getTriangles(), collision_time, U1, U2, omega1, omega2, initial_pos_1, initial_pos_2, contact_p_th, contact_index, contact_normal_th);
				cout << "final coll_time=" << collision_time << endl;
				sec = vvr::getSeconds() - sec;
				cout << "Found collision time in " << sec << " seconds." << endl;
			}
		}
		//m_style_flag |= FLAG(MOVE_B3);
		FLAG_OFF(m_style_flag, RUN_B3);

	}

	if (FLAG_ON(m_style_flag, MOVE_B3)) {
		m_anim.update(true);
		if (t >= collision_time) {
			cout << "reached collision" << endl;
			cout << "time=" << t << endl;
			cout << "theoretical time =" << collision_time << endl;
			if (!m_anim.paused()) {
				m_anim.pause();
			}
			vector<int> tr_inds;
			if ((obj_names[tempNo - 1] == "cube") || (option_B3 == 1) || (option_B3 == 2)) {
				contact_p = vec(model_vertices[contact_index].x, model_vertices[contact_index].y, model_vertices[contact_index].z);
				find_Normal_Point(contact_p, contact_normal, model_vertices, m_model.getTriangles(), tr_inds);
				contact_normal *= -1;
				contact_normal_th *= -1;
			}
			else {
				contact_p = vec(temp_vertices[contact_index].x, temp_vertices[contact_index].y, temp_vertices[contact_index].z);
				find_Normal_Point(contact_p, contact_normal, temp_vertices, temp_B1.getTriangles(), tr_inds);
			}

			FLAG_OFF(m_style_flag, MOVE_B3);
			m_style_flag |= FLAG(SHOW_B3);
			//m_style_flag |= FLAG(RUN_B4);

		}
	}

	if (FLAG_ON(m_style_flag, SHOW_B4)) {
		Point3D(contact_p_th.x, contact_p_th.y, contact_p_th.z, Colour::magenta).draw();
		vec x_n = contact_p_th + jr * 10;
		LineSeg3D(contact_p_th.x, contact_p_th.y, contact_p_th.z, x_n.x, x_n.y, x_n.z, Colour::yellowGreen).draw();
		//m_anim.update(true);
		//FLAG_OFF(m_style_flag, SHOW_B4);
	}
	if (FLAG_ON(m_style_flag, RUN_B4)) {
		Find_CM(model_vertices, m_center_mass);
		Find_CM(temp_vertices, B1_CM);
		r1 = contact_p_th - B1_CM;
		r2 = contact_p_th - m_center_mass;
		Reaction_Impulse_Vector(jr_mag, jr, e, Ur, U1, U2, omega1, omega2, contact_normal_th, m1, m2, r1, r2, temp_vertices, model_vertices);
		vec jr_angles = vec((jr.AngleBetween(vec(1, 0, 0)) * 180) / pi, (jr.AngleBetween(vec(0, 1, 0)) * 180) / pi, (jr.AngleBetween(vec(0, 0, 1)) * 180) / pi);
		cout << "jr_magnitude=" << abs(jr_mag) << endl << "jr_angles with x,y,z = (" << jr_angles.x << "," << jr_angles.y << "," << jr_angles.z << ")" << endl;
		Print_Vec(jr, "jr");
		FLAG_OFF(m_style_flag, RUN_B4);
		m_style_flag |= FLAG(SHOW_B4);
	}
	//SHOWS
	if (FLAG_ON(m_style_flag, SHOW_PLANE)) {
		vvr::Colour colPlane(0x41, 0x14, 0xB3);
		float u = 20, v = 20;
		math::vec p0(m_plane.Point(-u, -v, math::vec(0, 0, 0)));
		math::vec p1(m_plane.Point(-u, v, math::vec(0, 0, 0)));
		math::vec p2(m_plane.Point(u, -v, math::vec(0, 0, 0)));
		math::vec p3(m_plane.Point(u, v, math::vec(0, 0, 0)));
		math2vvr(math::Triangle(p0, p1, p2), colPlane).draw();
		math2vvr(math::Triangle(p2, p1, p3), colPlane).draw();
	}

	if (FLAG_ON(m_style_flag, SHOW_SOLID)) {
		m_model.draw(m_obj_col, SOLID);
		//m_style_flag &= !FLAG(SOLID);

	}
	if (FLAG_ON(m_style_flag, SHOW_WIRE)) {
		m_model.draw(Colour::black, WIRE);
		//m_style_flag &= !FLAG(WIRE);
	}
	if (FLAG_ON(m_style_flag, SHOW_NORMALS)) {
		m_model.draw(Colour::black, NORMALS);
		//m_style_flag &= !FLAG(NORMALS);
	}
	if (FLAG_ON(m_style_flag, SHOW_AXES)) {
		Mesh original = Mesh(objFile);
		original.draw(Colour::black, AXES);
		//m_style_flag &= !FLAG(AXES);

	}
	if (FLAG_ON(m_style_flag, SHOW_TEMP_AABB)) {
		vvr::Colour colPlane(0x41, 0x14, 0xB3);
		vec c1 = t_AABB.minPoint;
		vec c2 = t_AABB.maxPoint;
		vvr::Box3D box(c1.x, c1.y, c1.z, c2.x, c2.y, c2.z);
		box.setTransparency(1);
		box.setColour(vvr::Colour::cyan);
		box.draw();
		for (int i = 0; i < 8; i++) {
			Point3D(t_AABB.CornerPoint(i).x, t_AABB.CornerPoint(i).y, t_AABB.CornerPoint(i).z, vvr::Colour::blue).draw();
		}
	}
	if (FLAG_ON(m_style_flag, SHOW_AABB)) {
		vvr::Colour colPlane(0x41, 0x14, 0xB3);
		vec c1 = m_AABB.minPoint;
		vec c2 = m_AABB.maxPoint;
		vvr::Box3D box(c1.x, c1.y, c1.z, c2.x, c2.y, c2.z);
		box.setTransparency(1);
		box.setColour(vvr::Colour::cyan);
		box.draw();
		for (int i = 0; i < 8; i++) {
			Point3D(m_AABB.CornerPoint(i).x, m_AABB.CornerPoint(i).y, m_AABB.CornerPoint(i).z, vvr::Colour::blue).draw();
		}
	}
	if (FLAG_ON(m_style_flag, SHOW_TEMP_SPHERE)) {
		vvr::Sphere3D sphere(t_sphere.Centroid().x, t_sphere.Centroid().y, t_sphere.Centroid().z, t_sphere.r);
		sphere.setSolidRender(false);
		sphere.setColour(vvr::Colour::cyan);
		sphere.draw();
	}
	if (FLAG_ON(m_style_flag, SHOW_SPHERE)) {
		vvr::Sphere3D sphere(m_sphere.Centroid().x, m_sphere.Centroid().y, m_sphere.Centroid().z, m_sphere.r);
		sphere.setSolidRender(false);
		sphere.setColour(vvr::Colour::cyan);
		sphere.draw();
	}
	if (FLAG_ON(m_style_flag, TEMP_SOLID)) {
		temp_B1.draw(m_obj_col, SOLID);
		temp_B1.draw(Colour::black, WIRE);
	}

	if (FLAG_ON(m_style_flag, TEMP_WIRE)) {
		temp_B1.draw(Colour::black, WIRE);
	}
	//! Draw intersecting triangles of model
	vector<vvr::Triangle>& triangles = m_model.getTriangles();
	for (int i = 0; i < m_intersections.size(); i++) {
		vvr::Triangle& t = triangles[m_intersections[i]];
		Triangle3D t3d(
			t.v1().x, t.v1().y, t.v1().z,
			t.v2().x, t.v2().y, t.v2().z,
			t.v3().x, t.v3().y, t.v3().z,
			Colour::green);
		t3d.draw();
	}

	//! Compute & Display FPS
	static float last_update = 0;
	static float last_show = 0;
	const float sec = vvr::getSeconds();
	const float dt = sec - last_update;
	const float dt_show = sec - last_show;
	int FPS = 1.0 / dt;
	last_update = sec;
	if (dt_show >= 1) {
		//echo(FPS);
		//last_show = sec;
	}

}

void Mesh3DScene::sliderChanged(int slider_id, float v)
{
	switch (slider_id)
	{
	case 0:
		m_anim.setSpeed(v);
		break;
	}
}

int main(int argc, char* argv[])
{
	try {
		string path = argv[0];
		PROJECT_ROOT = path.substr(0, 46);
		cout << PROJECT_ROOT << endl;
		return vvr::mainLoop(argc, argv, new Mesh3DScene);
	}
	catch (std::string exc) {
		cerr << exc << endl;
		return 1;
	}
	catch (...)
	{
		cerr << "Unknown exception" << endl;
		return 1;
	}
}

//! Ground::
Grounds::Grounds()
	:m_col()
{
}
Grounds::Grounds(const float W, const float D, const float B, const float T, const vvr::Colour& colour)
	: m_col(colour)
{
	const vec vA(-W / 2, B, -D / 2);
	const vec vB(+W / 2, B, -D / 2);
	const vec vC(+W / 2, B, +D / 2);
	const vec vD(-W / 2, B, +D / 2);
	const vec vE(-W / 2, T, -D / 2);
	const vec vF(+W / 2, T, -D / 2);

	m_floor_tris.push_back(math::Triangle(vB, vA, vD));
	m_floor_tris.push_back(math::Triangle(vB, vD, vC));
	m_floor_tris.push_back(math::Triangle(vF, vE, vA));
	m_floor_tris.push_back(math::Triangle(vF, vA, vB));
}

void Grounds::draw() const
{
	for (int i = 0; i < m_floor_tris.size(); i++)
	{
		vvr::Triangle3D floor_tri = vvr::math2vvr(m_floor_tris.at(i), m_col);
		floor_tri.setSolidRender(true);
		floor_tri.draw();
	}
}

void find_L_Matrix(vector<vvr::Triangle>& m_triangles, SparseMatrix<float>& L, MatrixXf& Coords) {
	int verticesCount = Coords.rows();
	int trianglesCount = m_triangles.size();

	SparseMatrix<float> A(verticesCount, verticesCount);
	SparseMatrix<float> I(verticesCount, verticesCount);
	SparseMatrix<float> D(verticesCount, verticesCount);
	SparseMatrix<float> D_inverse(verticesCount, verticesCount);

	SparseIdentity(I, verticesCount);

	// give values to A and D


	for (int i = 0; i < trianglesCount; i++) {
		for (int j = 0; j < 2; j++) {
			vector<int> inds;
			if (j == 0) { inds = { 1,2 }; }
			else if (j == 1) { inds = { 0,2 }; }
			else { inds = { 0,1 }; }
			if (A.coeffRef(m_triangles[i].v[j], m_triangles[i].v[inds[0]]) == 0) {
				A.coeffRef(m_triangles[i].v[j], m_triangles[i].v[inds[0]]) = 1;
				A.coeffRef(m_triangles[i].v[inds[0]], m_triangles[i].v[j]) = 1;
				D.coeffRef(m_triangles[i].v[j], m_triangles[i].v[j])++;
				D.coeffRef(m_triangles[i].v[inds[0]], m_triangles[i].v[inds[0]])++;
			}
			if (A.coeffRef(m_triangles[i].v[j], m_triangles[i].v[inds[1]]) == 0) {
				A.coeffRef(m_triangles[i].v[j], m_triangles[i].v[inds[1]]) = 1;
				A.coeffRef(m_triangles[i].v[inds[1]], m_triangles[i].v[j]) = 1;
				D.coeffRef(m_triangles[i].v[j], m_triangles[i].v[j])++;
				D.coeffRef(m_triangles[i].v[inds[1]], m_triangles[i].v[inds[1]])++;
			}
		}
	}
	///  The inverse of matrix D will also be a diagonal nxn matrix:

	SparseDiagonalInverse(D, D_inverse, verticesCount);

	///  find L 
	L = I - D_inverse * A;
}

void ReplaceVertices(std::vector<vec>& vertices, MatrixXf& DifCoords)
{
	for (int i = 0; i < vertices.size(); i++) {
		vertices[i] = vec(DifCoords(i, 0), DifCoords(i, 1), DifCoords(i, 2));
	}
}

void Task_A_1_Find_Differential(SparseMatrix<float>& L, MatrixXf& Coords, MatrixXf& DifCoords) {
	DifCoords = L * Coords;
}
void Task_A2_Smoothing(SparseMatrix<float>& L, MatrixXf& Coords, MatrixXf& DifCoords, int iter, float lambda) {
	SparseMatrix<float> I(Coords.rows(), Coords.rows());
	SparseIdentity(I, Coords.rows());
	DifCoords = Coords;
	for (int i = 0; i < iter; i++) {
		DifCoords = (I - lambda * L) * DifCoords;
	}
}

void Task_A3_Taubin_Smoothing(SparseMatrix<float>& L, MatrixXf& Coords, MatrixXf& DifCoords, int iter, float lambda, float me) {
	SparseMatrix<float> I(Coords.rows(), Coords.rows());
	SparseIdentity(I, Coords.rows());
	DifCoords = Coords;
	for (int i = 0; i < iter; i++) {
		DifCoords = (I - lambda * L) * DifCoords;
		DifCoords = (I + me * L) * DifCoords;
	}
}


bool compareIndSum(vector<int> t1, vector<int> t2) {
	int indsum1 = t1[0] + t1[1] + t1[2];
	int indsum2 = t2[0] + t2[1] + t2[2];
	return(indsum1 < indsum2);
}

void SparseIdentity(SparseMatrix<float>& I, int n) {
	I.reserve(VectorXi::Constant(n, 1));
	for (int i = 0; i < n; i++) {
		I.insert(i, i) = 1;
	}
}

void SparseDiagonalInverse(SparseMatrix<float>& D, SparseMatrix<float>& D_inverse, int n) {
	for (int i = 0; i < n; i++) {
		D_inverse.coeffRef(i, i) = 1 / D.coeffRef(i, i);
	}
}

void Find_CM(vector<vec>& vertices, vec& cm) {
	int l = vertices.size();
	cm = vec(0, 0, 0);
	for (int i = 0; i < l; i++) {
		cm += vertices[i];
	}
	cm.x /= l;
	cm.y /= l;
	cm.z /= l;
}

void findSigned_Brute(std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles) {
	int v_count = vertices.size();
	int o_count = origin_vertices.size();
	//float task_dur = vvr::getSeconds();
	for (int j = 0; j < o_count; j++) {
		vec p = vec(origin_vertices[j].x, origin_vertices[j].y, origin_vertices[j].z);
		float best_dist = inf;
		int ind = 0;
		for (int i = 0; i < v_count; i++) {
			float dist = p.DistanceSq(vertices[i]);
			if (dist < best_dist) {
				best_dist = dist;
				ind = i;
			}
		}
		signed_distance sd;
		//check if lineSeg that starts from an outside point and ends at ovj 
		//intersects with an odd or even number of mesh-triangles
		if (meshContainsPoint(triangles, p, vertices)) {
			//inside the mesh signed distance is negative
			sd.d = -sqrt(best_dist);
		}
		else {
			//outside the mesh it is positive
			sd.d = sqrt(best_dist);
		}
		sd.p = vec(p.x, p.y, p.z);
		sd.vi = ind;
		signed_distance_function.push_back(sd);
	}
	//task_dur = vvr::getSeconds() - task_dur;
	//std::cout << "task time=" << task_dur << endl;
}

void findSigned_KDTree(KDTree* m_KDTree, std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles) {
	int v_count = vertices.size();
	int o_count = origin_vertices.size();
	//float task_dur = vvr::getSeconds();
	for (int j = 0; j < o_count; j++) {
		vec p = vec(origin_vertices[j].x, origin_vertices[j].y, origin_vertices[j].z);
		float best_dist;
		const KDNode* nearest = NULL;
		FindNearestNeighboor(p, m_KDTree->root(), &nearest, &best_dist);
		vec nn = nearest->split_point;
		signed_distance sd;
		//check if lineSeg that starts from an outside point and ends at ovj 
		//intersects with an odd or even number of mesh-triangles
		if (meshContainsPoint(triangles, p, vertices)) {
			//inside the mesh signed distance is negative
			sd.d = -sqrt(best_dist);
		}
		else {
			//outside the mesh it is positive
			sd.d = sqrt(best_dist);
		}
		sd.p = vec(p.x, p.y, p.z);
		sd.vi = j;
		signed_distance_function.push_back(sd);
	}
	//task_dur = vvr::getSeconds() - task_dur;
	//std::cout << "task time=" << task_dur << endl;
}

void findSigned_Cube(std::vector<vec>& cube_vertices, vector<vvr::Triangle>& cube_triangles, math::float3x4& transform_m, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles) {
	vec cm;
	Find_CM(cube_vertices, cm);
	//vector from a cube vertex to its center
	vec cm_p = cm - cube_vertices[0];
	//length of cube edge 
	float l = sqrt(2 * cm_p.LengthSq());
	//half-length
	float a = l / 2;
	//rotation matrix of cube
	Matrix3f rotation_m = Matrix3f();
	rotation_m(0, 0) = transform_m.At(0, 0);
	rotation_m(0, 1) = transform_m.At(0, 1);
	rotation_m(0, 2) = transform_m.At(0, 2);
	rotation_m(1, 0) = transform_m.At(1, 0);
	rotation_m(1, 1) = transform_m.At(1, 1);
	rotation_m(1, 2) = transform_m.At(1, 2);
	rotation_m(2, 0) = transform_m.At(2, 0);
	rotation_m(2, 1) = transform_m.At(2, 1);
	rotation_m(2, 2) = transform_m.At(2, 2);
	Matrix3f rotation_m_inv = rotation_m.inverse();
	vec u = vec(rotation_m_inv(0, 0), rotation_m_inv(1, 0), rotation_m_inv(2, 0));
	vec v = vec(rotation_m_inv(0, 1), rotation_m_inv(1, 1), rotation_m_inv(2, 1));
	vec w = vec(rotation_m_inv(0, 2), rotation_m_inv(1, 2), rotation_m_inv(2, 2));

	vec inter_point;
	for (int i = 0; i < vertices.size(); i++) {
		//transfer and transform point to cube position and local-axis
		// p_local = (p_global - cm)*RM_INV
		vec p = vertices[i] - cm;
		float x = (u.x * p.x + v.x * p.y + w.x * p.z) / a;
		float y = (u.y * p.x + v.y * p.y + w.y * p.z) / a;
		float z = (u.z * p.x + v.z * p.y + w.z * p.z) / a;
		float d = 0;

		//find transformed-point's distance to unit,axis aligned cube with points (+-1,+-1,+-1)
		if (abs(x) <= 1) {
			if (abs(y) <= 1) {
				d = abs(z) - 1;
			}
			else {
				if (abs(z) <= 1) {
					d = abs(y) - 1;

				}
				else {
					d = sqrt(pow((abs(y) - 1), 2) + pow((abs(z) - 1), 2));
				}
			}
		}
		else {
			if (abs(y) <= 1) {
				if (abs(z) <= 1) {
					d = abs(x) - 1;
				}
				else {
					d = sqrt(pow(abs(x) - 1, 2) + pow(abs(z) - 1, 2));
				}
			}
			else {
				if (abs(z) <= 1) {
					d = sqrt(pow(abs(x) - 1, 2) + pow(abs(y) - 1, 2));
				}
				else {
					d = sqrt(pow(abs(x) - 1, 2) + pow(abs(y) - 1, 2) + pow(abs(z) - 1, 2));
				}
			}
		}
		//scale back to cube's original size
		d *= a;
		signed_distance sd;
		sd.d = d;
		sd.vi = i;
		sd.p = vec(vertices[i].x, vertices[i].y, vertices[i].z);
		signed_distance_function.push_back(sd);
	}


}

void findSigned_AABB(math::AABB aabb, std::vector<vec>& aabb_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function) {
	vec cm;
	Find_CM(aabb_vertices, cm);
	float lx = abs(aabb.MaxX() - aabb.MinX());
	float ly = abs(aabb.MaxY() - aabb.MinY());
	float lz = abs(aabb.MaxZ() - aabb.MinZ());
	float ax = lx / 2;
	float ay = ly / 2;
	float az = lz / 2;
	for (int i = 0; i < vertices.size(); i++) {
		//transfer and transform point to cube position and local-axis
		// p_local = (p_global - cm)*RM_INV
		vec p = vertices[i] - cm;
		float x = p.x / ax;
		float y = p.y / ay;
		float z = p.z / az;
		float d = 0;
		//find transformed-point's distance to unit,axis aligned cube with points (+-1,+-1,+-1)
		if (abs(x) <= 1) {
			if (abs(y) <= 1) {
				d = abs(z) - 1;
				//scale back distance to rectangle with points (+-ax,+-ay,+-az)
				d *= az;
			}
			else {
				if (abs(z) <= 1) {
					d = abs(y) - 1;
					d *= ay;
				}
				else {
					d = sqrt(pow((ay * (abs(y) - 1)), 2) + pow((az * (abs(z) - 1)), 2));
				}
			}
		}
		else {
			if (abs(y) <= 1) {
				if (abs(z) <= 1) {
					d = abs(x) - 1;
					d *= ax;
				}
				else {
					d = sqrt(pow(ax * (abs(x) - 1), 2) + pow(az * (abs(z) - 1), 2));
				}
			}
			else {
				if (abs(z) <= 1) {
					d = sqrt(pow(ax * (abs(x) - 1), 2) + pow(ay * (abs(y) - 1), 2));
				}
				else {
					d = sqrt(pow(ax * (abs(x) - 1), 2) + pow(ay * (abs(y) - 1), 2) + pow(az * (abs(z) - 1), 2));
				}
			}
		}

		signed_distance sd;
		sd.d = d;
		sd.vi = i;
		sd.p = vec(vertices[i].x, vertices[i].y, vertices[i].z);
		signed_distance_function.push_back(sd);
		//cout << "found aabb sdf" << endl;
	}
}

void findSigned_Sphere(math::Sphere& sphere, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function) {
	vec cm = vec(sphere.Centroid().x, sphere.Centroid().y, sphere.Centroid().z);
	float r = sphere.r;
	for (int i = 0; i < vertices.size(); i++) {
		vec pcm = cm - vertices[i];
		float d = pcm.Length() - r;
		signed_distance sd;
		sd.d = d;
		sd.vi = i;
		sd.p = vec(vertices[i].x, vertices[i].y, vertices[i].z);
		signed_distance_function.push_back(sd);
	}
}

bool cubeContainsPoint(vec vertice, std::vector<vec>& cube_vertices, math::float3x4& transform_m) {
	vec cm;
	Find_CM(cube_vertices, cm);
	//vector from a cube vertex to its center
	vec cm_p = cm - cube_vertices[0];
	//length of cube edge 
	float l = sqrt(2 * cm_p.LengthSq());
	//half-length
	float a = l / 2;
	//rotation matrix of cube
	Matrix3f rotation_m = Matrix3f();
	rotation_m(0, 0) = transform_m.At(0, 0);
	rotation_m(0, 1) = transform_m.At(0, 1);
	rotation_m(0, 2) = transform_m.At(0, 2);
	rotation_m(1, 0) = transform_m.At(1, 0);
	rotation_m(1, 1) = transform_m.At(1, 1);
	rotation_m(1, 2) = transform_m.At(1, 2);
	rotation_m(2, 0) = transform_m.At(2, 0);
	rotation_m(2, 1) = transform_m.At(2, 1);
	rotation_m(2, 2) = transform_m.At(2, 2);
	Matrix3f rotation_m_inv = rotation_m.inverse();
	vec u = vec(rotation_m_inv(0, 0), rotation_m_inv(1, 0), rotation_m_inv(2, 0));
	vec v = vec(rotation_m_inv(0, 1), rotation_m_inv(1, 1), rotation_m_inv(2, 1));
	vec w = vec(rotation_m_inv(0, 2), rotation_m_inv(1, 2), rotation_m_inv(2, 2));
	vec p = vertice - cm;
	float x = (u.x * p.x + v.x * p.y + w.x * p.z) / a;
	float y = (u.y * p.x + v.y * p.y + w.y * p.z) / a;
	float z = (u.z * p.x + v.z * p.y + w.z * p.z) / a;
	//check if point is inside the cube
	if (abs(x) <= 1 && abs(y) <= 1 && abs(z) <= 1) {
		return true;
	}
	return false;
}

bool AABBContainsPoint(vec vertice, math::AABB& aabb) {
	vec cm = aabb.Centroid();
	vec p = vertice - cm;
	float lx = abs(aabb.MaxX() - aabb.MinX());
	float ly = abs(aabb.MaxY() - aabb.MinY());
	float lz = abs(aabb.MaxZ() - aabb.MinZ());
	float ax = lx / 2;
	float ay = ly / 2;
	float az = lz / 2;
	float x = p.x / ax;
	float y = p.y / ay;
	float z = p.z / az;
	if (abs(x) <= 1 && abs(y) <= 1 && abs(z) <= 1) {
		return true;
	}
	return false;
}

bool sphereContainsPoint(vec vertice, math::Sphere& sphere) {
	return ((sphere.Centroid() - vertice).LengthSq() <= (sphere.r * sphere.r));
}

bool Distcomparator(diffDistance d1, diffDistance d2) {
	return d1.dist < d2.dist;
}

lineEquation CreateLineEq(const vvr::LineSeg3D line) {
	vec p1 = vec(line.x1, line.y1, line.z1);
	vec p2 = vec(line.x2, line.y2, line.z2);
	vec u = p2 - p1;
	lineEquation lineEq;
	lineEq.line = line;
	lineEq.p1 = p1;
	lineEq.p2 = p2;
	lineEq.u = u;
	lineEq.A = -u.x / u.y;
	lineEq.B = -p1.x + (u.x * p1.y) / u.y;
	lineEq.C = -u.x / u.z;
	lineEq.D = (u.x / u.z) * p1.z - p1.x;
	return lineEq;
}

bool lineContainsPoint(vvr::LineSeg3D line, vec point) {
	float length = sqrt((line.x1 - line.x2) * (line.x1 - line.x2) + (line.y1 - line.y2) * (line.y1 - line.y2) + (line.z1 - line.z2) * (line.z1 - line.z2));
	float oDist = sqrt((point.x - line.x1) * (point.x - line.x1) + (point.y - line.y1) * (point.y - line.y1) + (point.z - line.z1) * (point.z - line.z1));
	float eDist = sqrt((point.x - line.x2) * (point.x - line.x2) + (point.y - line.y2) * (point.y - line.y2) + (point.z - line.z2) * (point.z - line.z2));
	float total = oDist + eDist;
	if (total <= length + 0.0001 && total >= length - 0.0001) {
		return true;
	}
	else return false;
}

void lineIntersection(const vvr::LineSeg3D line1, vvr::Triangle triangle, vec& point, vector<vec>& vertices) {
	//outside points on the given line
	lineEquation lEq = CreateLineEq(line1);
	vec p1 = vertices[triangle.v[0]];
	vec p2 = vertices[triangle.v[1]];
	vec p3 = vertices[triangle.v[2]];
	vec l1 = p2 - p1;
	vec l2 = p3 - p1;
	vec N = math::Cross(l1, l2);
	float d = -Dot(lEq.u, N);
	vec p1_Lp1 = lEq.p1 - p1;
	vec p1_Lp1xu = math::Cross(p1_Lp1, lEq.u);
	float u = Dot(l2, p1_Lp1xu) / d;
	float v = -Dot(l1, p1_Lp1xu) / d;
	float t = Dot(p1_Lp1, N) / d;
	float uv = u + v;
	bool res = (fabs(d) >= 1e-6 && u >= 0.0 && v >= 0.0 && uv <= 1.0);
	if (res) {
		point = lEq.p1 + t * lEq.u;
	}
	else {
		point = vec(inf, inf, inf);
	}
}

double SignedVolume(vec a, vec b, vec c, vec d) {
	vec e = math::Cross(b - a, c - a);
	return  Dot(e, d - a) / 6.0;
}

bool meshContainsPoint(std::vector<vvr::Triangle>& triangles, vec p, vector<vec>& vertices) {
	vec out = vec(-100, -100, -100);
	LineSeg3D line = LineSeg3D(out.x, out.y, out.z, p.x, p.y, p.z, vvr::Colour::black);
	int noF_inters = 0;
	for (int i = 0; i < triangles.size(); i++) {
		vec inter;
		lineIntersection(line, triangles[i], inter, vertices);
		if (inter.x != inf && lineContainsPoint(line, inter)) {
			noF_inters++;
		}
	}
	if (noF_inters % 2 != 0) {
		return true;
	}
	else {
		return false;
	}
}

void FindNearestNeighboor(const vec& test_pt, const KDNode* root, const KDNode** nn, float* best_dist) {
	if (!root) return;

	//! Distances
	const double d = test_pt.DistanceSq(root->split_point);
	const double d_split = root->split_point.ptr()[root->axis] - test_pt.ptr()[root->axis];
	const bool right_of_split = d_split <= 0;

	if (*nn == NULL || d < *best_dist) {
		*best_dist = d;
		*nn = root;
	}
	//check best direction recursively
	if (right_of_split) {
		FindNearestNeighboor(test_pt, root->child_right, nn, best_dist);
	}
	else {
		FindNearestNeighboor(test_pt, root->child_left, nn, best_dist);
	}
	//decide if we need to check the other direction
	if (SQUARE(d_split) >= *best_dist) return;
	//check other direction
	if (!right_of_split) {
		FindNearestNeighboor(test_pt, root->child_right, nn, best_dist);
	}
	else {
		FindNearestNeighboor(test_pt, root->child_left, nn, best_dist);
	}
}

//! KDTree::

KDTree::KDTree(VecArray& pts)
	: pts(pts)
{
	const float t = vvr::getSeconds();
	m_root = new KDNode();
	m_depth = makeNode(m_root, pts, 0);
	const float KDTree_construction_time = vvr::getSeconds() - t;
	//echo(KDTree_construction_time);
	//echo(m_depth);
}

KDTree::~KDTree()
{
	const float t = vvr::getSeconds();
	delete m_root;
	const float KDTree_destruction_time = vvr::getSeconds() - t;
	//echo(KDTree_destruction_time);
}

int KDTree::makeNode(KDNode* node, VecArray& pts, const int level)
{
	//! Sort along the appropriate axis, find median point and split.
	const int axis = level % DIMENSIONS;
	std::sort(pts.begin(), pts.end(), VecComparator(axis));
	const int i_median = pts.size() / 2;

	//! Set node members
	node->level = level;
	node->axis = axis;
	node->split_point = pts[i_median];
	node->aabb.SetFrom(&pts[0], pts.size());

	//! Continue recursively or stop.
	if (pts.size() <= 1)
	{
		return level;
	}
	else
	{
		int level_left = 0;
		int level_right = 0;
		VecArray pts_left(pts.begin(), pts.begin() + i_median);
		VecArray pts_right(pts.begin() + i_median + 1, pts.end());

		if (!pts_left.empty())
		{
			node->child_left = new KDNode();
			level_left = makeNode(node->child_left, pts_left, level + 1);

		}
		if (!pts_right.empty())
		{
			node->child_right = new KDNode();
			level_right = makeNode(node->child_right, pts_right, level + 1);
		}

		int max_level = std::max(level_left, level_right);
		return max_level;
	}
}

void KDTree::getNodesOfLevel(KDNode* node, std::vector<KDNode*>& nodes, int level)
{
	if (!level)
	{
		nodes.push_back(node);
	}
	else
	{
		if (node->child_left) getNodesOfLevel(node->child_left, nodes, level - 1);
		if (node->child_right) getNodesOfLevel(node->child_right, nodes, level - 1);
	}
}

std::vector<KDNode*> KDTree::getNodesOfLevel(const int level)
{
	std::vector<KDNode*> nodes;
	if (!m_root) return nodes;
	getNodesOfLevel(m_root, nodes, level);
	return nodes;
}

float delta(int i, int j) {
	if (i == j) return 1.0;
	else return 0.0;
}

void Inertia_Tensor(Matrix3f& I, float mk, std::vector<vec>& vertices) {
	vec cm;
	Find_CM(vertices, cm);
	for (int k = 0; k < vertices.size(); k++) {
		vec rk = vertices[k] - cm;
		float rkSq = rk.LengthSq();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				I(i, j) += mk * (rkSq * delta(i, j) - rk.ptr()[i] * rk.ptr()[j]);
			}
		}
	}

}

void Inertia_Tensor_Reverse(Matrix3f& I, Matrix3f& I_Rev) {
	I_Rev = I.inverse();
}

void Reaction_Impulse_Vector(float& jr_mag, vec& jr, float e, vec& Ur, vec& U1, vec& U2, vec& omega1, vec& omega2, vec n, float m1, float m2, vec r1, vec r2, std::vector<vec> vertices1, std::vector<vec> vertices2) {
	cout << "old speeds:" << "U1=" << U1 << ", U2=" << U2 << endl;
	cout << "old angular speeds:" << "OMEGA1=" << omega1 << ", OMEGA2=" << omega2 << endl;
	n /= n.Length();
	vec Up1 = U1 + math::Cross(omega1, r1);
	vec Up2 = U2 + math::Cross(omega2, r2);
	Ur = Up2 - Up1;
	float mk1 = m1 / vertices1.size();
	float mk2 = m2 / vertices2.size();
	Matrix3f I1, I2, I1_Rev, I2_Rev;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			I1(i, j) = 0;
			I2(i, j) = 0;
		}
	}
	Inertia_Tensor(I1, mk1, vertices1);
	Inertia_Tensor(I2, mk2, vertices2);
	Inertia_Tensor_Reverse(I1, I1_Rev);
	Inertia_Tensor_Reverse(I2, I2_Rev);
	vec temp1 = math::Cross(math::Cross(r1, n), r1);
	vec temp2 = math::Cross(math::Cross(r2, n), r2);
	Vector3f T1 = Vector3f(temp1.x, temp1.y, temp1.z);
	Vector3f T2 = Vector3f(temp2.x, temp2.y, temp2.z);
	Vector3f T = I1_Rev * T1 + I2_Rev * T2;
	vec temp = vec(T(0), T(1), T(2));
	cout << "Ur*n=" << Dot(Ur, n) << ", temp*n=" << Dot(temp, n) << endl;
	jr_mag = (-(1 + e) * Dot(Ur, n)) / ((1 / m1) + (1 / m2) + Dot(temp, n));
	jr = jr_mag * n;
	//new velocities
	U1 = U1 - (jr_mag / m1) * n;
	U2 = U2 + (jr_mag / m2) * n;
	//new angular velocities
	vec r1xn = math::Cross(r1, n);
	vec r2xn = math::Cross(r2, n);
	Vector3f R1N = Vector3f(r1xn.x, r1xn.y, r1xn.z);
	Vector3f R2N = Vector3f(r2xn.x, r2xn.y, r2xn.z);
	Vector3f PL1 = I1_Rev * R1N;
	Vector3f PL2 = I2_Rev * R2N;
	vec pl1 = vec(PL1(0), PL1(1), PL1(2));
	vec pl2 = vec(PL2(0), PL2(1), PL2(2));
	omega1 = omega1 - jr_mag * pl1;
	omega2 = omega2 - jr_mag * pl2;
	cout << "new speeds:" << "U1'=" << U1 << ", U2'=" << U2 << endl;
	cout << "new angular speeds:" << "OMEGA1'=" << omega1 << ", OMEGA2'=" << omega2 << endl;
}

void find_Normals(std::vector<normal_point>& normals_list, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles) {
	vector<vec> tr_normals;
	vector<vector<float>> weights;
	vector<vec> normals;
	vector<vector<int>> trs_ind_array;
	for (int i = 0; i < vertices.size(); i++) {
		normals.push_back(vec(0, 0, 0));
		vector<int>temp;
		vector<float> temp_f;
		trs_ind_array.push_back(temp);
		weights.push_back(temp_f);
	}
	for (int j = 0; j < triangles.size(); j++) {
		vvr::Triangle& t = triangles[j];
		vector<vec> tr_points;
		for (int k = 0; k < 3; k++) {
			vec l = vec(vertices[t.v[k]].x, vertices[t.v[k]].y, vertices[t.v[k]].z);
			tr_points.push_back(l);
		}
		vec A = tr_points[1] - tr_points[0];
		vec B = tr_points[2] - tr_points[0];
		vec nj = math::Cross(A, B);
		nj /= nj.Length();
		tr_normals.push_back(nj);
		for (int k = 0; k < 3; k++) {
			trs_ind_array[t.v[k]].push_back(j);
			int c1, c2;
			if (k == 0) {
				c1 = 1;
				c2 = 2;
			}
			else if (k == 1) {
				c1 = 2;
				c2 = 0;
			}
			else if (k == 2) {
				c1 = 0;
				c2 = 1;
			}
			vec pp1 = tr_points[c1] - tr_points[k];
			vec pp2 = tr_points[c2] - tr_points[k];
			//float th = pp1.AngleBetween(pp2);
			float costh = (pp1.x * pp2.x + pp1.y * pp2.y + pp1.z * pp2.z) / (pp1.Length() * pp2.Length());
			float th = math::Acos(costh);
			//float wj = 1.0;
			float wj = sin(th) / (pp1.Length() * pp2.Length());
			weights[t.v[k]].push_back(wj);
			normals[t.v[k]] += wj * tr_normals[j];
		}
	}
	for (int g = 0; g < signed_distance_function.size(); g++) {
		float d = signed_distance_function[g].d;
		if (d < 0) {
			int i = signed_distance_function[g].vi;
			normal_point n;
			vec normal = vec(normals[i].x, normals[i].y, normals[i].z);
			float weight_sum = 0;
			for (int f = 0; f < weights[i].size(); f++) {
				weight_sum += weights[i][f];
			}
			normal /= weight_sum;
			n.n = vec(normal.x, normal.y, normal.z);
			n.n *= -1;
			n.v = i;
			n.sd.d = d;
			n.sd.vi = i;
			n.sd.p = vec(vertices[i].x, vertices[i].y, vertices[i].z);
			for (int k = 0; k < trs_ind_array[i].size(); k++) {
				n.trs.push_back(trs_ind_array[i][k]);
			}
			vec x_n = vertices[i] + (n.n / n.n.Length()) * 0.1;
			if (meshContainsPoint(triangles, x_n, vertices)) {
				if (vertices.size() != 24) {
					n.n *= -1;
				}
			}
			normals_list.push_back(n);
		}
	}
	std::cout << normals_list.size() << endl;
}

void find_Normals_all(std::vector<normal_point>& normals_list, std::vector<vec>& vertices, std::vector<vvr::Triangle>& triangles) {
	vector<vec> tr_normals;
	vector<vector<float>> weights;
	vector<vec> normals;
	vector<vector<int>> trs_ind_array;
	for (int i = 0; i < vertices.size(); i++) {
		normals.push_back(vec(0, 0, 0));
		vector<int>temp;
		vector<float> temp_f;
		trs_ind_array.push_back(temp);
		weights.push_back(temp_f);
	}
	for (int j = 0; j < triangles.size(); j++) {
		vvr::Triangle& t = triangles[j];
		vector<vec> tr_points;
		for (int k = 0; k < 3; k++) {
			vec l = vec(vertices[t.v[k]].x, vertices[t.v[k]].y, vertices[t.v[k]].z);
			tr_points.push_back(l);
		}
		vec A = tr_points[1] - tr_points[0];
		vec B = tr_points[2] - tr_points[0];
		vec nj = math::Cross(A, B);
		nj /= nj.Length();
		tr_normals.push_back(nj);
		for (int k = 0; k < 3; k++) {
			trs_ind_array[t.v[k]].push_back(j);
			int c1, c2;
			if (k == 0) {
				c1 = 1;
				c2 = 2;
			}
			else if (k == 1) {
				c1 = 2;
				c2 = 0;
			}
			else if (k == 2) {
				c1 = 0;
				c2 = 1;
			}
			vec pp1 = tr_points[c1] - tr_points[k];
			vec pp2 = tr_points[c2] - tr_points[k];
			//float th = pp1.AngleBetween(pp2);
			float costh = (pp1.x * pp2.x + pp1.y * pp2.y + pp1.z * pp2.z) / (pp1.Length() * pp2.Length());
			float th = math::Acos(costh);
			//float wj = 1.0;
			float wj = sin(th) / (pp1.Length() * pp2.Length());
			weights[t.v[k]].push_back(wj);
			normals[t.v[k]] += wj * tr_normals[j];
		}
	}
	for (int i = 0; i < vertices.size(); i++) {
		normal_point n;
		vec normal = vec(normals[i].x, normals[i].y, normals[i].z);
		float weight_sum = 0;
		for (int f = 0; f < weights[i].size(); f++) {
			weight_sum += weights[i][f];
		}
		normal /= weight_sum;
		n.n = vec(normal.x, normal.y, normal.z);
		n.n *= -1;
		n.v = i;
		for (int k = 0; k < trs_ind_array[i].size(); k++) {
			n.trs.push_back(trs_ind_array[i][k]);
		}
		vec x_n = vertices[i] + (n.n / n.n.Length()) * 0.1;
		if (meshContainsPoint(triangles, x_n, vertices)) {
			if (vertices.size() != 24) {
				n.n *= -1;
			}
		}
		normals_list.push_back(n);
	}
	std::cout << normals_list.size() << endl;
}

void find_Normal_Point(vec p, vec& normal, std::vector<vec>& vertices, std::vector<vvr::Triangle>& triangles, std::vector<int>& tr_inds) {
	vector<vector<long double>> tr_normals;
	vector<vector<long double>> weights;
	vector<vector<long double>> normals;
	vector<vector<int>> trs_ind_array;
	for (int i = 0; i < vertices.size(); i++) {
		vector<int>temp;
		vector<long double> temp_f;
		trs_ind_array.push_back(temp);
		weights.push_back(temp_f);
		temp_f.push_back(0.0);
		temp_f.push_back(0.0);
		temp_f.push_back(0.0);
		normals.push_back(temp_f);
	}
	for (int j = 0; j < triangles.size(); j++) {
		vvr::Triangle& t = triangles[j];
		vector<vec> tr_points;
		for (int k = 0; k < 3; k++) {
			vec l = vec(vertices[t.v[k]].x, vertices[t.v[k]].y, vertices[t.v[k]].z);
			tr_points.push_back(l);
		}
		vec A = tr_points[1] - tr_points[0];
		vec B = tr_points[2] - tr_points[0];
		long double njx = long double(A.y) * long double(B.z) - long double(A.z) * long double(B.y);
		long double njy = long double(A.z) * long double(B.x) - long double(A.x) * long double(B.z);
		long double njz = long double(A.x) * long double(B.y) - long double(A.y) * long double(B.x);
		long double njL = sqrt(pow(njx, 2) + pow(njy, 2) + pow(njz, 2));
		njx /= njL;
		njy /= njL;
		njz /= njL;
		vector<long double> nj;
		nj.push_back(njx);
		nj.push_back(njy);
		nj.push_back(njz);
		tr_normals.push_back(nj);
		for (int k = 0; k < 3; k++) {
			trs_ind_array[t.v[k]].push_back(j);
			int c1, c2;
			if (k == 0) {
				c1 = 1;
				c2 = 2;
			}
			else if (k == 1) {
				c1 = 2;
				c2 = 0;
			}
			else if (k == 2) {
				c1 = 0;
				c2 = 1;
			}
			vec pp1 = tr_points[c1] - tr_points[k];
			vec pp2 = tr_points[c2] - tr_points[k];
			long double pp1L = sqrt(pow(long double(pp1.x), 2) + pow(long double(pp1.y), 2) + pow(long double(pp1.z), 2));
			long double pp2L = sqrt(pow(long double(pp2.x), 2) + pow(long double(pp2.y), 2) + pow(long double(pp2.z), 2));
			long double nom = long double(pp1.x) * long double(pp2.x) + long double(pp1.y) * long double(pp2.y) + long double(pp1.z) * long double(pp2.z);
			long double dem = pp1L * pp2L;
			long double costh = nom / dem;
			long double th = acos(costh);
			//long double wj = 1.0;
			long double wj = sin(th) / dem;
			weights[t.v[k]].push_back(wj);
			normals[t.v[k]][0] += wj * tr_normals[j][0];
			normals[t.v[k]][1] += wj * tr_normals[j][1];
			normals[t.v[k]][2] += wj * tr_normals[j][2];
		}
	}
	for (int i = 0; i < vertices.size(); i++) {
		if (p.x == vertices[i].x && p.y == vertices[i].y && p.z == vertices[i].z) {
			long double weight_sum = 0;
			for (int f = 0; f < weights[i].size(); f++) {
				weight_sum += weights[i][f];
			}
			long double temp_x = normals[i][0] / weight_sum;
			long double temp_y = normals[i][1] / weight_sum;
			long double temp_z = normals[i][2] / weight_sum;
			long double nL = sqrt(pow(temp_x, 2) + pow(temp_y, 2) + pow(temp_z, 2));
			normal = vec(temp_x, temp_y, temp_z);
			normal *= -1;
			vec x_n = p + (normal / nL) * 0.1;
			if (meshContainsPoint(triangles, x_n, vertices)) {
				normal *= -1;
			}
			for (int d = 0; d < trs_ind_array[i].size(); d++) {
				tr_inds.push_back(trs_ind_array[i][d]);
			}
			break;
		}
	}
}

void Converge_Collision(std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& origin_triangles, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal) {
	bool cont = true;
	float c = 1.0;
	int it = 0;
	float prev_sdf = inf;
	float prev_col_time = 0;
	float prev_prev_col_time = 0;
	float change = 0;
	float l_bound = -inf;
	float u_bound = inf;
	VecArray pts1, pts2;
	KDTree* KDTree1 = NULL;
	KDTree* KDTree2 = NULL;
	vector<vec> origin_vertices_t;
	vector<vec> vertices_t;
	//collision_time = 0;
	while (cont) {
		signed_distance_function.clear();
		vector<vvr::Triangle> triangles_t, origin_triangles_t;
		math::float3x4 final_transform;
		moveVertices(origin_vertices, collision_time, Uo, omegao, final_transform, origin_vertices_t, in_poso);
		moveVertices(vertices, collision_time, U, omega, final_transform, vertices_t, in_pos);
		copyTriangles(vertices_t, triangles, triangles_t);
		updateTrees(KDTree1, KDTree2, pts1, pts2, vertices_t, origin_vertices_t);
		findSigned_KDTree(KDTree1, origin_vertices_t, vertices_t, signed_distance_function, triangles_t);
		sort(signed_distance_function.begin(), signed_distance_function.end(), sdf_comp);
		float min_sdf = signed_distance_function[0].d;
		contact_index = signed_distance_function[0].vi;
		int prev_index = signed_distance_function[0].vi;
		if (min_sdf < 0) {
			float step_sdf = min_sdf;
			for (int i = 1; i < signed_distance_function.size(); i++) {
				if (step_sdf * signed_distance_function[i].d < 0) {
					min_sdf = min(abs(step_sdf), abs(signed_distance_function[i].d));
					if (min_sdf == abs(step_sdf)) {
						min_sdf = step_sdf;
						contact_index = prev_index;
					}
					else {
						min_sdf = signed_distance_function[i].d;
						contact_index = signed_distance_function[i].vi;
					}
					break;
				}
				step_sdf = signed_distance_function[i].d;
				prev_index = signed_distance_function[i].vi;
			}
		}
		if (abs(min_sdf) > (abs(prev_sdf) * 2) && min_sdf* prev_sdf > 0) {
			cout << "min_sdf_wrong=" << min_sdf << ",wrong_col_time=" << collision_time << endl;;
			if (min_sdf < 0) {
				l_bound = collision_time + 1e-3;
				collision_time += change;
			}
			else {
				u_bound = collision_time - 1e-3;
				collision_time -= change;
			}

		}
		else {
			change = abs(min_sdf) / ((U - Uo).Length() * 8);
			if (prev_sdf * min_sdf < 0) {
				if (prev_prev_col_time > prev_col_time) {
					l_bound = prev_col_time;
					u_bound = prev_prev_col_time;
				}
				else {
					u_bound = prev_col_time;
					l_bound = prev_prev_col_time;
				}
				change = (u_bound - l_bound) / 2;
			}
			if (min_sdf > 0.1 && (change > 1e-2 || it < 5)) {
				while (collision_time + change > u_bound) {
					change /= 2;
				}
				collision_time += change;


			}
			else if (min_sdf < -0.1 && (change > 1e-2 || it < 5)) {
				while (collision_time - change < l_bound) {
					change /= 2;
				}
				collision_time -= change;

			}
			else {
				cout << "min_sdf_final=" << min_sdf;
				cout << " , final_change=" << change << endl;
				cont = false;
				copyTriangles(origin_vertices_t, origin_triangles, origin_triangles_t);
				cout << "contact index=" << contact_index << " , x=" << origin_vertices_t[contact_index].x << " ,y=" << origin_vertices_t[contact_index].y << " ,z=" << origin_vertices_t[contact_index].z << endl;
				cout << "length=" << origin_vertices_t.size() << endl;
				contact_p = vec(origin_vertices_t[contact_index].x, origin_vertices_t[contact_index].y, origin_vertices_t[contact_index].z);
				vector<int> trs;
				find_Normal_Point(contact_p, contact_normal, origin_vertices_t, origin_triangles_t, trs);
			}
			prev_sdf = min_sdf;
			prev_prev_col_time = prev_col_time;
			prev_col_time = collision_time;
			//cout << "min_sdf" << it << "=" << min_sdf << ",col_time=" << collision_time << endl;
			it++;
		}
	}
}

void Converge_Collision_SIREN(std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& origin_triangles, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal) {
	bool cont = true;
	float c = 1.0;
	int it = 0;
	float prev_sdf = inf;
	float prev_col_time = 0;
	float prev_prev_col_time = 0;
	float change = 0;
	float l_bound = -inf;
	float u_bound = inf;
	VecArray pts1, pts2;
	KDTree* KDTree1 = NULL;
	KDTree* KDTree2 = NULL;
	vector<vec> origin_vertices_t;
	string script_path = PROJECT_ROOT + "/test/siren/sdf_model_create.py";
	//string script_path = PROJECT_ROOT + "/test/siren/dist/sdf_model_create.exe";
	vector<vec> vertices_t;
	//collision_time = 0;
	while (cont) {
		signed_distance_function.clear();
		vector<vvr::Triangle> triangles_t, origin_triangles_t;
		math::float3x4 final_transform;
		moveVertices(origin_vertices, collision_time, Uo, omegao, final_transform, origin_vertices_t, in_poso);
		moveVertices(vertices, collision_time, U, omega, final_transform, vertices_t, in_pos);
		copyTriangles(vertices_t, triangles, triangles_t);
		updateTrees(KDTree1, KDTree2, pts1, pts2, vertices_t, origin_vertices_t);
		savePointsToText(origin_vertices_t);
		system(script_path.c_str());
		getSdfFromText(signed_distance_function, origin_vertices_t);
		sort(signed_distance_function.begin(), signed_distance_function.end(), sdf_comp);
		float min_sdf = signed_distance_function[0].d;
		contact_index = signed_distance_function[0].vi;
		int prev_index = signed_distance_function[0].vi;
		if (min_sdf < 0) {
			float step_sdf = min_sdf;
			for (int i = 1; i < signed_distance_function.size(); i++) {
				if (step_sdf * signed_distance_function[i].d < 0) {
					min_sdf = min(abs(step_sdf), abs(signed_distance_function[i].d));
					if (min_sdf == abs(step_sdf)) {
						min_sdf = step_sdf;
						contact_index = prev_index;
					}
					else {
						min_sdf = signed_distance_function[i].d;
						contact_index = signed_distance_function[i].vi;
					}
					break;
				}
				step_sdf = signed_distance_function[i].d;
				prev_index = signed_distance_function[i].vi;
			}
		}
		if (abs(min_sdf) > (abs(prev_sdf) * 2) && min_sdf* prev_sdf > 0) {
			cout << "min_sdf_wrong=" << min_sdf << ",wrong_col_time=" << collision_time << endl;;
			if (min_sdf < 0) {
				l_bound = collision_time + 1e-3;
				collision_time += change;
			}
			else {
				u_bound = collision_time - 1e-3;
				collision_time -= change;
			}

		}
		else {
			change = abs(min_sdf) / ((U - Uo).Length() * 8);
			if (prev_sdf * min_sdf < 0) {
				if (prev_prev_col_time > prev_col_time) {
					l_bound = prev_col_time;
					u_bound = prev_prev_col_time;
				}
				else {
					u_bound = prev_col_time;
					l_bound = prev_prev_col_time;
				}
				change = (u_bound - l_bound) / 2;
			}
			if (min_sdf > 0.01 && (change > 1e-3 || it < 5)) {
				while (collision_time + change > u_bound) {
					change /= 2;
				}
				collision_time += change;


			}
			else if (min_sdf < -0.01 && (change > 1e-3 || it < 5)) {
				while (collision_time - change < l_bound) {
					change /= 2;
				}
				collision_time -= change;

			}
			else {
				cout << "min_sdf_final=" << min_sdf;
				cout << " , final_change=" << change << endl;
				cont = false;
				copyTriangles(origin_vertices_t, origin_triangles, origin_triangles_t);
				cout << "contact index=" << contact_index << " , x=" << origin_vertices_t[contact_index].x << " ,y=" << origin_vertices_t[contact_index].y << " ,z=" << origin_vertices_t[contact_index].z << endl;
				cout << "length=" << origin_vertices_t.size() << endl;
				contact_p = vec(origin_vertices_t[contact_index].x, origin_vertices_t[contact_index].y, origin_vertices_t[contact_index].z);
				vector<int> trs;
				find_Normal_Point(contact_p, contact_normal, origin_vertices_t, origin_triangles_t, trs);
			}
			prev_sdf = min_sdf;
			prev_prev_col_time = prev_col_time;
			prev_col_time = collision_time;
			//cout << "min_sdf" << it << "=" << min_sdf << ",col_time=" << collision_time << endl;
			it++;
		}
	}
}


void Converge_Collision_Cube(vvr::Mesh& origin_model, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal) {
	bool cont = true;
	float c = 1.0;
	int it = 0;
	float prev_sdf = inf;
	float prev_col_time = 0;
	float prev_prev_col_time = 0;
	float change = 0;
	float l_bound = -inf;
	float u_bound = inf;
	VecArray pts1, pts2;
	KDTree* KDTree1 = NULL;
	KDTree* KDTree2 = NULL;
	vector<vec> origin_vertices_t;
	vector<vec> vertices_t;
	//collision_time = 0;
	while (cont) {
		signed_distance_function.clear();
		vector<vvr::Triangle> origin_triangles_t, triangles_t;
		math::float3x4 final_transform;
		moveVertices(origin_model.getVertices(), collision_time, Uo, omegao, final_transform, origin_vertices_t, in_poso);
		moveVertices(vertices, collision_time, U, omega, final_transform, vertices_t, in_pos);
		copyTriangles(vertices_t, triangles, triangles_t);
		copyTriangles(origin_vertices_t, origin_model.getTriangles(), origin_triangles_t);
		findSigned_Cube(origin_vertices_t, origin_triangles_t, final_transform, vertices_t, signed_distance_function, triangles_t);
		sort(signed_distance_function.begin(), signed_distance_function.end(), sdf_comp);
		float min_sdf = signed_distance_function[0].d;
		contact_index = signed_distance_function[0].vi;
		int prev_index = signed_distance_function[0].vi;
		if (min_sdf < 0) {
			float step_sdf = min_sdf;
			for (int i = 1; i < signed_distance_function.size(); i++) {
				if (step_sdf * signed_distance_function[i].d < 0) {
					min_sdf = min(abs(step_sdf), abs(signed_distance_function[i].d));
					if (min_sdf == abs(step_sdf)) {
						min_sdf = step_sdf;
						contact_index = prev_index;
					}
					else {
						min_sdf = signed_distance_function[i].d;
						contact_index = signed_distance_function[i].vi;
					}
					break;
				}
				step_sdf = signed_distance_function[i].d;
				prev_index = signed_distance_function[i].vi;
			}
		}
		if (abs(min_sdf) > (abs(prev_sdf) * 2) && min_sdf* prev_sdf > 0) {
			cout << "min_sdf_wrong=" << min_sdf << ",wrong_col_time=" << collision_time << endl;;
			if (min_sdf < 0) {
				l_bound = collision_time + 1e-3;
				collision_time += change;
			}
			else {
				u_bound = collision_time - 1e-3;
				collision_time -= change;
			}

		}
		else {
			change = abs(min_sdf) / ((U - Uo).Length() * 8);
			if (prev_sdf * min_sdf < 0) {
				if (prev_prev_col_time > prev_col_time) {
					l_bound = prev_col_time;
					u_bound = prev_prev_col_time;
				}
				else {
					u_bound = prev_col_time;
					l_bound = prev_prev_col_time;
				}
				change = (u_bound - l_bound) / 2;
			}
			if (min_sdf > 0.01 && (change > 1e-3 || it < 5)) {
				while (collision_time + change > u_bound) {
					change /= 2;
				}
				collision_time += change;


			}
			else if (min_sdf < -0.01 && (change > 1e-3 || it < 5)) {
				while (collision_time - change < l_bound) {
					change /= 2;
				}
				collision_time -= change;

			}
			else {
				cout << "min_sdf_final=" << min_sdf;
				cout << " , final_change=" << change << endl;
				cont = false;
				cout << "theoretical contact index=" << contact_index << " , x=" << vertices_t[contact_index].x << " ,y=" << vertices_t[contact_index].y << " ,z=" << vertices_t[contact_index].z << endl;
				contact_p = vec(vertices_t[contact_index].x, vertices_t[contact_index].y, vertices_t[contact_index].z);
				vector<int> trs;
				find_Normal_Point(contact_p, contact_normal, vertices_t, triangles_t, trs);
			}
			prev_sdf = min_sdf;
			prev_prev_col_time = prev_col_time;
			prev_col_time = collision_time;
			it++;
		}
	}
}

void Converge_Collision_Sphere(vvr::Mesh& origin_model, std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal) {
	bool cont = true;
	float c = 1.0;
	int it = 0;
	bool first_time = true;
	float prev_sdf = inf;
	float prev_col_time = 0;
	float prev_prev_col_time = 0;
	float change = 0;
	float l_bound = -inf;
	float u_bound = inf;
	VecArray pts1, pts2;
	vector<vec> origin_vertices_t;
	vector<vec> vertices_t;
	math::Sphere& sphere = Sphere();
	//collision_time = 0;
	while (cont) {
		signed_distance_function.clear();
		vector<vvr::Triangle> triangles_t;
		math::float3x4 final_transform;
		moveVertices(origin_vertices, collision_time, Uo, omegao, final_transform, origin_vertices_t, in_poso);
		moveVertices(vertices, collision_time, U, omega, final_transform, vertices_t, in_pos);
		findSphere(origin_vertices_t, sphere);
		findSigned_Sphere(sphere, vertices_t, signed_distance_function);
		sort(signed_distance_function.begin(), signed_distance_function.end(), sdf_comp);
		float min_sdf = signed_distance_function[0].d;
		contact_index = signed_distance_function[0].vi;
		int prev_index = signed_distance_function[0].vi;
		if (min_sdf < 0) {
			float step_sdf = min_sdf;
			for (int i = 1; i < signed_distance_function.size(); i++) {
				if (step_sdf * signed_distance_function[i].d < 0) {
					min_sdf = min(abs(step_sdf), abs(signed_distance_function[i].d));
					if (min_sdf == abs(step_sdf)) {
						min_sdf = step_sdf;
						contact_index = prev_index;
					}
					else {
						min_sdf = signed_distance_function[i].d;
						contact_index = signed_distance_function[i].vi;
					}
					break;
				}
				step_sdf = signed_distance_function[i].d;
				prev_index = signed_distance_function[i].vi;
			}
		}
		if (abs(min_sdf) > (abs(prev_sdf) * 2) && min_sdf* prev_sdf > 0) {
			cout << "min_sdf_wrong=" << min_sdf << ",wrong_col_time=" << collision_time << endl;;
			if (min_sdf < 0) {
				l_bound = collision_time + 1e-3;
				collision_time += change;
			}
			else {
				u_bound = collision_time - 1e-3;
				collision_time -= change;
			}

		}
		else {
			change = abs(min_sdf) / ((U - Uo).Length() * 8);
			if (prev_sdf * min_sdf < 0) {
				if (prev_prev_col_time > prev_col_time) {
					l_bound = prev_col_time;
					u_bound = prev_prev_col_time;
				}
				else {
					u_bound = prev_col_time;
					l_bound = prev_prev_col_time;
				}
				change = (u_bound - l_bound) / 2;
			}
			if (min_sdf > 0.001 && (change > 1e-4 || it < 5)) {
				while (collision_time + change > u_bound) {
					change /= 2;
				}
				collision_time += change;


			}
			else if (min_sdf < -0.001 && (change > 1e-4 || it < 5)) {
				while (collision_time - change < l_bound) {
					change /= 2;
				}
				collision_time -= change;

			}
			else {
				cout << "min_sdf_final=" << min_sdf;
				cout << " , final_change=" << change << endl;
				cont = false;
				cout << "theoretical contact index=" << contact_index << " , x=" << vertices_t[contact_index].x << " ,y=" << vertices_t[contact_index].y << " ,z=" << vertices_t[contact_index].z << endl;
				contact_p = vec(vertices_t[contact_index].x, vertices_t[contact_index].y, vertices_t[contact_index].z);
				vector<int> trs;
				copyTriangles(vertices_t, triangles, triangles_t);
				find_Normal_Point(contact_p, contact_normal, vertices_t, triangles_t, trs);
			}
			prev_sdf = min_sdf;
			prev_prev_col_time = prev_col_time;
			prev_col_time = collision_time;
			it++;
		}
	}
}

void Converge_Collision_AABB(vvr::Mesh& origin_model, std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal) {
	bool cont = true;
	float c = 1.0;
	int it = 0;
	bool first_time = true;
	float prev_sdf = inf;
	float prev_col_time = 0;
	float prev_prev_col_time = 0;
	float change = 0;
	float l_bound = -inf;
	float u_bound = inf;
	VecArray pts1, pts2;
	vector<vec> origin_vertices_t;
	vector<vec> vertices_t;
	vector<vec> aabb_vertices;
	math::AABB& aabb = AABB();
	//collision_time = 0;
	while (cont) {
		signed_distance_function.clear();
		aabb_vertices.clear();
		vector<vvr::Triangle> triangles_t;
		math::float3x4 final_transform;
		moveVertices(origin_vertices, collision_time, Uo, omegao, final_transform, origin_vertices_t, in_poso);
		moveVertices(vertices, collision_time, U, omega, final_transform, vertices_t, in_pos);
		findAABB(origin_vertices_t, aabb);
		for (int i = 0; i < 8; i++) {
			aabb_vertices.push_back(aabb.CornerPoint(i));
		}
		findSigned_AABB(aabb, aabb_vertices, vertices_t, signed_distance_function);
		sort(signed_distance_function.begin(), signed_distance_function.end(), sdf_comp);
		float min_sdf = signed_distance_function[0].d;
		contact_index = signed_distance_function[0].vi;
		int prev_index = signed_distance_function[0].vi;
		if (min_sdf < 0) {
			float step_sdf = min_sdf;
			for (int i = 1; i < signed_distance_function.size(); i++) {
				if (step_sdf * signed_distance_function[i].d < 0) {
					min_sdf = min(abs(step_sdf), abs(signed_distance_function[i].d));
					if (min_sdf == abs(step_sdf)) {
						min_sdf = step_sdf;
						contact_index = prev_index;
					}
					else {
						min_sdf = signed_distance_function[i].d;
						contact_index = signed_distance_function[i].vi;
					}
					break;
				}
				step_sdf = signed_distance_function[i].d;
				prev_index = signed_distance_function[i].vi;
			}
		}
		if (abs(min_sdf) > (abs(prev_sdf) * 2) && min_sdf* prev_sdf > 0) {
			cout << "min_sdf_wrong=" << min_sdf << ",wrong_col_time=" << collision_time << endl;;
			if (min_sdf < 0) {
				l_bound = collision_time + 1e-3;
				collision_time += change;
			}
			else {
				u_bound = collision_time - 1e-3;
				collision_time -= change;
			}

		}
		else {
			change = abs(min_sdf) / ((U - Uo).Length() * 8);
			if (prev_sdf * min_sdf < 0) {
				if (prev_prev_col_time > prev_col_time) {
					l_bound = prev_col_time;
					u_bound = prev_prev_col_time;
				}
				else {
					u_bound = prev_col_time;
					l_bound = prev_prev_col_time;
				}
				change = (u_bound - l_bound) / 2;
			}
			if (min_sdf > 0.001 && (change > 1e-4 || it < 5)) {
				while (collision_time + change > u_bound) {
					change /= 2;
				}
				collision_time += change;


			}
			else if (min_sdf < -0.001 && (change > 1e-4 || it < 5)) {
				while (collision_time - change < l_bound) {
					change /= 2;
				}
				collision_time -= change;

			}
			else {
				cout << "min_sdf_final=" << min_sdf;
				cout << " , final_change=" << change << endl;
				cont = false;
				cout << "theoretical contact index=" << contact_index << " , x=" << vertices_t[contact_index].x << " ,y=" << vertices_t[contact_index].y << " ,z=" << vertices_t[contact_index].z << endl;
				contact_p = vec(vertices_t[contact_index].x, vertices_t[contact_index].y, vertices_t[contact_index].z);
				vector<int> trs;
				copyTriangles(vertices_t, triangles, triangles_t);
				find_Normal_Point(contact_p, contact_normal, vertices_t, triangles_t, trs);
			}
			prev_sdf = min_sdf;
			prev_prev_col_time = prev_col_time;
			prev_col_time = collision_time;
			it++;
		}
	}
}

void moveVertices(std::vector<vec>& vertices, float t, vec U, vec omega, math::float3x4& final_transform, std::vector<vec>& new_vertices, vec in_pos) {
	new_vertices.clear();
	vec dl = vec(t * U.x, t * U.y, t * U.z);
	vec dth = vec(t * omega.x, t * omega.y, t * omega.z);
	math::float3x3 r_x = float3x3(1, 0, 0, 0, cos(dth.x), -sin(dth.x), 0, sin(dth.x), cos(dth.x));
	math::float3x3 r_y = float3x3(cos(dth.y), 0, sin(dth.y), 0, 1, 0, -sin(dth.y), 0, cos(dth.y));
	math::float3x3 r_z = float3x3(cos(dth.z), -sin(dth.z), 0, sin(dth.z), cos(dth.z), 0, 0, 0, 1);
	math::float3x3 rm = r_z * r_y * r_x;
	math::float3x3 transf_cords = float3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
	math::float3x3 new_transf_cords = transf_cords * rm;
	vec axes1 = vec(in_pos.x + dl.x, in_pos.y + dl.y, in_pos.z + dl.z);
	final_transform = float3x4(new_transf_cords.At(0, 0), new_transf_cords.At(0, 1), new_transf_cords.At(0, 2), axes1.x, new_transf_cords.At(1, 0), new_transf_cords.At(1, 1), new_transf_cords.At(1, 2), axes1.y, new_transf_cords.At(2, 0), new_transf_cords.At(2, 1), new_transf_cords.At(2, 2), axes1.z);
	vector<math::float3x3> t_vertices;
	for (int i = 0; i < vertices.size(); i++) {
		t_vertices.push_back(float3x3(vertices[i].x, 0, 0, vertices[i].y, 0, 0, vertices[i].z, 0, 0));
		t_vertices[i] = new_transf_cords * t_vertices[i];
	}
	for (int i = 0; i < vertices.size(); i++) {
		new_vertices.push_back(vec(t_vertices[i].At(0, 0), t_vertices[i].At(1, 0), t_vertices[i].At(2, 0)));
		new_vertices[i] += axes1;
	}
}

bool sdf_comp(signed_distance d1, signed_distance d2) {
	return d1.d < d2.d;
}

void copyVertices(std::vector<vec>& v, std::vector<vec>& v_t) {
	for (int i = 0; i < v.size(); i++) {
		v_t.push_back(vec(v[i].x, v[i].y, v[i].z));
	}
}

void copyTriangles(std::vector<vec>& v, std::vector<vvr::Triangle>& t, std::vector<vvr::Triangle>& t_t) {
	for (int i = 0; i < t.size(); i++) {
		vvr::Triangle& triangle = t[i];
		t_t.push_back(vvr::Triangle(&v, t[i].v[0], t[i].v[1], t[i].v[2]));
	}
}

void updateTrees(KDTree*& m_KDTree, KDTree*& temp_KDTree, VecArray& m_pts, VecArray& temp_pts, vector<vec>& model_vertices, vector<vec>& temp_vertices) {
	delete m_KDTree;
	delete temp_KDTree;
	m_pts.clear();
	temp_pts.clear();
	for (int i = 0; i < model_vertices.size(); i++) {
		m_pts.push_back(vec(model_vertices[i].x, model_vertices[i].y, model_vertices[i].z));
	}
	m_KDTree = new KDTree(m_pts);
	for (int i = 0; i < temp_vertices.size(); i++) {
		temp_pts.push_back(vec(temp_vertices[i].x, temp_vertices[i].y, temp_vertices[i].z));
	}
	temp_KDTree = new KDTree(temp_pts);
}

void find_Volume(std::vector<vec>& v, std::vector<vvr::Triangle>& triangles, float& V) {
	V = 0.0;
	for (int i = 0; i < triangles.size(); i++) {
		vvr::Triangle t = triangles[i];
		V += Dot(math::Cross(v[t.v[0]], v[t.v[1]]), v[t.v[2]]);
	}
	V = abs(V / 6.0);
}

void find_Mass(float V, float density, float& mass) {
	mass = V * density;
}

void Converge_Collision_Nosdf(std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<vvr::Triangle>& origin_triangles, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal) {
	bool cont = true;
	float change = 1e-3;
	vector<vec> origin_vertices_t;
	vector<vec> vertices_t;
	while (cont) {
		vector<vvr::Triangle> triangles_t, origin_triangles_t;
		math::float3x4 final_transform1, final_transform2;
		moveVertices(origin_vertices, collision_time, Uo, omegao, final_transform1, origin_vertices_t, in_poso);
		moveVertices(vertices, collision_time, U, omega, final_transform2, vertices_t, in_pos);
		copyTriangles(vertices_t, triangles, triangles_t);
		int o_count = origin_vertices_t.size();
		int v_count = vertices_t.size();
		for (int j = 0; j < o_count; j++) {
			vec p = vec(origin_vertices_t[j].x, origin_vertices_t[j].y, origin_vertices_t[j].z);
			bool inside = meshContainsPoint(triangles_t, p, vertices_t);
			if (inside && collision_time > 1) {
				cont = false;
				contact_p = vec(p.x, p.y, p.z);
				copyTriangles(origin_vertices_t, origin_triangles, origin_triangles_t);
				vector<int> trs;
				find_Normal_Point(contact_p, contact_normal, origin_vertices_t, origin_triangles_t, trs);
				contact_index = j;
			}
			else {
				collision_time += change;
			}
		}
	}
}

void Converge_Collision_Cube_Nosdf(vvr::Mesh& origin_model, std::vector<vec>& vertices, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal) {
	bool cont = true;
	float change = 1e-5;
	vector<vec> origin_vertices_t;
	vector<vec> vertices_t;
	while (cont) {
		vector<vvr::Triangle> triangles_t, origin_triangles_t;
		math::float3x4 final_transform1, final_transform2;
		moveVertices(origin_model.getVertices(), collision_time, Uo, omegao, final_transform1, origin_vertices_t, in_poso);
		moveVertices(vertices, collision_time, U, omega, final_transform2, vertices_t, in_pos);
		copyTriangles(vertices_t, triangles, triangles_t);
		copyTriangles(origin_vertices_t, origin_model.getTriangles(), origin_triangles_t);
		int o_count = origin_vertices_t.size();
		int v_count = vertices_t.size();
		for (int j = 0; j < v_count; j++) {
			vec p = vec(vertices_t[j].x, vertices_t[j].y, vertices_t[j].z);
			bool inside = cubeContainsPoint(p, origin_vertices_t, final_transform1);
			if (inside && collision_time > 1) {
				cont = false;
				contact_p = vec(p.x, p.y, p.z);
				vector<int> trs;
				find_Normal_Point(contact_p, contact_normal, vertices_t, triangles_t, trs);
				contact_index = j;
			}
			else {
				collision_time += change;
			}
		}
	}
}

void Converge_Collision_Sphere_Nosdf(vvr::Mesh& origin_model, std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal) {
	bool cont = true;
	float change = 1e-5;
	vector<vec> origin_vertices_t;
	vector<vec> vertices_t;
	math::Sphere& sphere = Sphere();
	while (cont) {
		vector<vvr::Triangle> triangles_t;
		math::float3x4 final_transform1, final_transform2;
		moveVertices(origin_vertices, collision_time, Uo, omegao, final_transform1, origin_vertices_t, in_poso);
		moveVertices(vertices, collision_time, U, omega, final_transform2, vertices_t, in_pos);
		findSphere(origin_vertices_t, sphere);
		copyTriangles(vertices_t, triangles, triangles_t);
		int o_count = origin_vertices_t.size();
		int v_count = vertices_t.size();
		for (int j = 0; j < v_count; j++) {
			vec p = vec(vertices_t[j].x, vertices_t[j].y, vertices_t[j].z);
			bool inside = sphereContainsPoint(p, sphere);
			if (inside && collision_time > 1) {
				cont = false;
				contact_p = vec(p.x, p.y, p.z);
				vector<int> trs;
				find_Normal_Point(contact_p, contact_normal, vertices_t, triangles_t, trs);
				contact_index = j;
			}
			else {
				collision_time += change;
			}
		}
	}
}


void Converge_Collision_AABB_Nosdf(vvr::Mesh& origin_model, std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal) {
	bool cont = true;
	float change = 1e-5;
	vector<vec> origin_vertices_t;
	vector<vec> vertices_t;
	vector<vec> aabb_vertices;
	math::AABB& aabb = AABB();
	while (cont) {
		aabb_vertices.clear();
		vector<vvr::Triangle> triangles_t;
		math::float3x4 final_transform1, final_transform2;
		moveVertices(origin_vertices, collision_time, Uo, omegao, final_transform1, origin_vertices_t, in_poso);
		moveVertices(vertices, collision_time, U, omega, final_transform2, vertices_t, in_pos);
		findAABB(origin_vertices_t, aabb);
		for (int i = 0; i < 8; i++) {
			aabb_vertices.push_back(aabb.CornerPoint(i));
		}
		copyTriangles(vertices_t, triangles, triangles_t);
		int o_count = origin_vertices_t.size();
		int v_count = vertices_t.size();
		for (int j = 0; j < v_count; j++) {
			vec p = vec(vertices_t[j].x, vertices_t[j].y, vertices_t[j].z);
			bool inside = AABBContainsPoint(p, aabb);
			if (inside && collision_time > 1) {
				cont = false;
				contact_p = vec(p.x, p.y, p.z);
				vector<int> trs;
				find_Normal_Point(contact_p, contact_normal, vertices_t, triangles_t, trs);
				contact_index = j;
			}
			else {
				collision_time += change;
			}
		}
	}
}


bool equal_Vec(vec v1, vec v2) {
	float diff_x = abs(v1.x - v2.x);
	float diff_y = abs(v1.y - v2.y);
	float diff_z = abs(v1.z - v2.z);
	if (diff_x < 1e-5 && diff_y < 1e-5 && diff_z < 1e-5) {
		return true;
	}
	return false;
}

void Print_Vec(vec v, string name) {
	cout << name << "=(" << v.x << "," << v.y << "," << v.z << ")" << endl;
}

void remove_dups(std::vector<vec>& vertices, std::vector<vvr::Triangle>& triangles) {
	vector<int> erased;
	for (int i = 0; i < vertices.size(); i++) {
		for (int j = i; j < vertices.size(); j++) {
			if (equal_Vec(vertices[i], vertices[j]) && i != j) {
				for (int k = 0; k < triangles.size(); k++) {
					vvr::Triangle& t = triangles[k];
					for (int f = 0; f < 3; f++) {
						if (t.v[f] == j) {
							t.v[f] = i;
						}
						else if (t.v[f] > j) {
							t.v[f]--;
						}
					}
				}
				vertices.erase(vertices.begin() + j);
				erased.push_back(j);
			}
		}
	}
	erased.clear();
	for (int i = 0; i < vertices.size(); i++) {
		for (int j = i; j < vertices.size(); j++) {
			if (equal_Vec(vertices[i], vertices[j]) && i != j) {
				for (int k = 0; k < triangles.size(); k++) {
					vvr::Triangle& t = triangles[k];
					for (int f = 0; f < 3; f++) {
						if (t.v[f] == j) {
							t.v[f] = i;
						}
						else if (t.v[f] > j) {
							t.v[f]--;
						}
					}
				}
				vertices.erase(vertices.begin() + j);
				erased.push_back(j);
			}
		}
	}
}

bool vec_Comp_X(vec v1, vec v2) {
	return (v1.x < v2.x);
}

bool vec_Comp_Y(vec v1, vec v2) {
	return (v1.y < v2.y);
}

bool vec_Comp_Z(vec v1, vec v2) {
	return (v1.z < v2.z);
}

void findAABB(std::vector<vec>& vertices, math::AABB& aabb) {
	vector<vec> vertices_t;
	copyVertices(vertices, vertices_t);
	sort(vertices_t.begin(), vertices_t.end(), vec_Comp_X);
	float Xmin = vertices_t[0].x;
	float Xmax = vertices_t[vertices_t.size() - 1].x;
	sort(vertices_t.begin(), vertices_t.end(), vec_Comp_Y);
	float Ymin = vertices_t[0].y;
	float Ymax = vertices_t[vertices_t.size() - 1].y;
	sort(vertices_t.begin(), vertices_t.end(), vec_Comp_Z);
	float Zmin = vertices_t[0].z;
	float Zmax = vertices_t[vertices_t.size() - 1].z;
	vec minP = vec(Xmin, Ymin, Zmin);
	vec maxP = vec(Xmax, Ymax, Zmax);
	aabb = AABB(minP, maxP);
}

void findSphere(std::vector<vec>& vertices, math::Sphere& sphere) {
	vec cm;
	Find_CM(vertices, cm);
	float dist = -inf;
	for (int i = 0; i < vertices.size(); i++) {
		float d = (vertices[i] - cm).Length();
		if (d > dist) dist = d;
	}
	sphere = Sphere(cm, dist);
}

void savePointsToText(vector<vec>& points) {
	std::ofstream ofs;
	
	//C: / Users / thanl / Documents / GEOMETRIA / 2022 / Model_load_with_Anim / resources /
	ofs.open(PROJECT_ROOT+"/test/siren/COLAB CODE/points.txt", std::ofstream::out | std::ofstream::trunc);
	int l = points.size();
	float scale = 24.31723f;
	for (int i = 0; i < l - 1; i++) {
		float x = (points[i].x + 4.5f) / scale;
		float y = points[i].y / scale;
		float z = (points[i].z + 3.8f) / scale;
		ofs << x << " " << y << " " << z << std::endl;
	}
	float x = (points[l - 1].x + 4.5f) / scale;
	float y = points[l - 1].y / scale;
	float z = (points[l - 1].z + 3.8f) / scale;
	ofs << x << " " << y << " " << z;
	ofs.close();
	cout << points.size() << "lines...done" << endl;


}

void getSdfFromText(std::vector<signed_distance>& sdf, std::vector<vec>& vertices) {
	signed_distance sd;
	std::ifstream infile(PROJECT_ROOT+"/test/siren/COLAB CODE/sdf.txt");
	float x;
	int count = 0;
	while (infile >> x) {
		sd.d = x * 24.31723;
		sd.vi = count;
		sd.p = vec(vertices[count].x, vertices[count].y, vertices[count].z);
		sdf.push_back(sd);
		count++;
	}
}