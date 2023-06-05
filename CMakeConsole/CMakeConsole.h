#include <VVRScene/canvas.h>
#include <VVRScene/mesh.h>
#include <VVRScene/settings.h>
#include <VVRScene/utils.h>
#include <VVRScene/scene.h>
#include <MathGeoLib.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <set>
#include "symmetriceigensolver3x3.h"
#include "canvas.h"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/SparseQR"
#include "Eigen/Eigenvalues"
#include "Eigen/SparseCholesky"
#include "math.h"
#include <filesystem>
#define FLAG(x) (1<<(x))
#define FLAG_ON(v,f) (v & FLAG(f))
#define FLAG_TOGGLE(v,c,f) case c: v ^= FLAG(f); std::cout \
    << #f << " = " << (FLAG_ON(v,f) ? "ON" : "OFF") \
    << std::endl; break
#define FLAG_OFF(v,f) if(FLAG_ON(v,f)) v-=FLAG(f); std::cout \
    << #f << " = " << (FLAG_ON(v,f) ? "ON" : "OFF") \
    << std::endl; 

void find_L_Matrix(std::vector<vvr::Triangle>& m_triangles, Eigen::SparseMatrix<float>& L, Eigen::MatrixXf& Coords);

void ReplaceVertices(std::vector<vec>& vertices, Eigen::MatrixXf& DifCoords);

void Task_A_1_Find_Differential(Eigen::SparseMatrix<float>& L, Eigen::MatrixXf& Coords, Eigen::MatrixXf& DifCoords);

void Task_A2_Smoothing(Eigen::SparseMatrix<float>& L, Eigen::MatrixXf& Coords, Eigen::MatrixXf& DifCoords, int iter, float lambda);

void Task_A3_Taubin_Smoothing(Eigen::SparseMatrix<float>& L, Eigen::MatrixXf& Coords, Eigen::MatrixXf& DifCoords, int iter, float lambda, float me);

bool compareIndSum(std::vector<int> t1, std::vector<int> t2);

void SparseIdentity(Eigen::SparseMatrix<float>& I, int n);

void SparseDiagonalInverse(Eigen::SparseMatrix<float>& D, Eigen::SparseMatrix<float>& D_inverse, int n);

void Find_CM(std::vector<vec>& vertices, vec& cm);

void Print_Vec(vec v, std::string name);

bool equal_Vec(vec v1, vec v2);

void remove_dups(std::vector<vec>& vertices, std::vector<vvr::Triangle>& triangles);

struct diffDistance {
	float dist;
	int index;
};
bool Distcomparator(diffDistance d1, diffDistance d2);
std::vector<diffDistance>Dist;
struct signed_distance {
	float d;
	vec p;
	int vi;
};

void findSigned_Brute(std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles);

struct lineEquation {
	vec p1;
	vec p2;
	vec u;
	float A;
	float B;
	float C;
	float D;
	vvr::LineSeg3D line;
};

lineEquation CreateLineEq(const vvr::LineSeg3D line);
void lineIntersection(const vvr::LineSeg3D line1, vvr::Triangle triangle, vec& point, std::vector<vec>& vertices);
bool lineContainsPoint(vvr::LineSeg3D line, vec point);
bool meshContainsPoint(std::vector<vvr::Triangle>& triangles, vec p, std::vector<vec>& vertices);
double SignedVolume(vec a, vec b, vec c, vec d);

/**
 * Function object to compare 2 3D-vecs in the specified axis.
 */
struct VecComparator {
	unsigned axis;
	VecComparator(unsigned axis) : axis(axis % 3) {}
	virtual inline bool operator() (const vec& v1, const vec& v2) {
		return (v1.ptr()[axis] < v2.ptr()[axis]);
	}
};

/**
 * A node of a KD-Tree
 */
struct KDNode
{
	vec split_point;
	int axis;
	int level;
	AABB aabb;
	KDNode* child_left;
	KDNode* child_right;
	KDNode() : child_left(NULL), child_right(NULL) {}
	~KDNode() { delete child_left; delete child_right; }
};

/**
 * KD-Tree wrapper. Holds a ptr to tree root.
 */
class KDTree
{
public:
	KDTree(VecArray& pts);
	~KDTree();
	std::vector<KDNode*> getNodesOfLevel(int level);
	int depth() const { return m_depth; }
	const KDNode* root() const { return m_root; }
	const VecArray& pts;

private:
	static int makeNode(KDNode* node, VecArray& pts, const int level);
	static void getNodesOfLevel(KDNode* node, std::vector<KDNode*>& nodes, int level);

private:
	KDNode* m_root;
	int m_depth;
};


/**
 * Find the nearest neighbour of `test_pt` inside `root`.
 */
void FindNearestNeighboor(const vec& test_pt, const KDNode* root, const KDNode** nn, float* best_dist);

void findSigned_KDTree(KDTree* m_KDTree, std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles);

void findSigned_Cube(std::vector<vec>& cube_vertices, std::vector<vvr::Triangle>& cube_triangles, math::float3x4& transform_m, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles);

void findSigned_AABB(math::AABB aabb, std::vector<vec>& aabb_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function);

void findSigned_Sphere(math::Sphere& sphere, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function);

bool cubeContainsPoint(vec vertice, std::vector<vec>& cube_vertices, math::float3x4& transform_m);

bool AABBContainsPoint(vec vertice, math::AABB& aabb);

bool sphereContainsPoint(vec vertice, math::Sphere& sphere);

float delta(int i, int j);

void Inertia_Tensor(Eigen::Matrix3f& I, float mk, std::vector<vec>& vertices);

void Inertia_Tensor_Reverse(Eigen::Matrix3f& I, Eigen::Matrix3f& I_Rev);

void Reaction_Impulse_Vector(float& jr_mag, vec& jr, float e, vec& Ur, vec& U1, vec& U2, vec& omega1, vec& omega2, vec n, float m1, float m2, vec r1, vec r2, std::vector<vec> vertices1, std::vector<vec> vertices2);

struct normal_point {
	vec n;
	int v;
	signed_distance sd;
	std::vector<int> trs;
};

void find_Normal_Point(vec p, vec& normal, std::vector<vec>& vertices, std::vector<vvr::Triangle>& triangles, std::vector<int>& tr_inds);

void find_Normals(std::vector<normal_point>& normals, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles);

void find_Normals_all(std::vector<normal_point>& normals, std::vector<vec>& vertices, std::vector<vvr::Triangle>& triangles);

void Converge_Collision(std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& origin_triangles, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal);

void Converge_Collision_Nosdf(std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<vvr::Triangle>& origin_triangles, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal);

void moveVertices(std::vector<vec>& vertices, float t, vec U, vec omega, math::float3x4& final_transform, std::vector<vec>& new_vertices, vec in_pos);

bool sdf_comp(signed_distance d1, signed_distance d2);

void copyVertices(std::vector<vec>& v, std::vector<vec>& v_t);

void copyTriangles(std::vector<vec>& v, std::vector<vvr::Triangle>& t, std::vector<vvr::Triangle>& t_t);

void updateTrees(KDTree*& m_KDTree, KDTree*& temp_KDTree, VecArray& m_pts, VecArray& temp_pts, std::vector<vec>& model_vertices, std::vector<vec>& temp_vertices);

void find_Volume(std::vector<vec>& v, std::vector<vvr::Triangle>& triangles, float& V);

void find_Mass(float V, float density, float& mass);

void Converge_Collision_Cube(vvr::Mesh& origin_model, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal);

void Converge_Collision_Cube_Nosdf(vvr::Mesh& origin_model, std::vector<vec>& vertices, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal);

void Converge_Collision_AABB(vvr::Mesh& origin_model, std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal);

void Converge_Collision_AABB_Nosdf(vvr::Mesh& origin_model, std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal);

void Converge_Collision_Sphere_Nosdf(vvr::Mesh& origin_model, std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal);

void Converge_Collision_Sphere(vvr::Mesh& origin_model, std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal);

void Converge_Collision_SIREN(std::vector<vec>& origin_vertices, std::vector<vec>& vertices, std::vector<signed_distance>& signed_distance_function, std::vector<vvr::Triangle>& origin_triangles, std::vector<vvr::Triangle>& triangles, float& collision_time, vec Uo, vec U, vec omegao, vec omega, vec in_poso, vec in_pos, vec& contact_p, int& contact_index, vec& contact_normal);

void findAABB(std::vector<vec>& vertices, math::AABB& aabb);

void findSphere(std::vector<vec>& vertices, math::Sphere& sphere);

bool vec_Comp_X(vec v1, vec v2);
bool vec_Comp_Y(vec v1, vec v2);
bool vec_Comp_Z(vec v1, vec v2);

void savePointsToText(std::vector<vec>& points);

void getSdfFromText(std::vector<signed_distance>& sdf, std::vector<vec>& vertices);

class Mesh3DScene : public vvr::Scene
{
	enum {
		SHOW_PLANE, SHOW_AXES, SHOW_SOLID, SHOW_WIRE, SHOW_NORMALS, RUN_A1, RUN_A2, RUN_A3, RUN_B1, RUN_B2, RUN_B3,
		SHOW_A1, SHOW_A2, SHOW_A3, SHOW_B1, TEMP_SOLID, LOAD_TEMP, RUN_B1_PLANE, SHOW_B1_PLANE, SHOW_B2, SHOW_B3,
		MOVE_B3, RUN_B4, SHOW_B4, TEMP_WIRE, SHOW_TEMP_AABB, SHOW_AABB, SHOW_TEMP_SPHERE, SHOW_SPHERE
	};
public:
	Mesh3DScene();
	const char* getName() const { return "3D Scene"; }
	void keyEvent(unsigned char key, bool up, int modif) override;
	void arrowEvent(vvr::ArrowDir dir, int modif) override;
	void sliderChanged(int slider_id, float val);

private:
	void draw() override;
	void reset() override;
	void resize() override;
	bool idle() override;
	void printKeyboardShortcuts();

private:
	//!!CHANGE BELOW TO THE ROOT DIRECTORY OF YOUR PROJECT:
	int objNo, tempNo, option_B1, option_B3;
	int m_style_flag;
	float m_plane_d;
	vvr::Canvas2D m_canvas;
	vvr::Colour m_obj_col;
	vvr::Mesh m_model_original, m_model, m_model_non_dup;
	vvr::Box3D m_aabb;
	math::vec m_center_mass;
	math::vec B1_CM;
	math::vec m_pca_cen;
	math::vec m_pca_dir;
	math::Plane m_plane;
	std::vector<int> m_intersections;
	vvr::Animation m_anim;
	float m_invalidation_sec;
	std::string objFile;
	math::AABB boundary;
	vvr::Mesh temp_A1, temp_A2, temp_A3, temp_B1, temp_B1_original, temp_B1_non_dup;
	std::vector<int>qDist;
	std::string cubeFile;
	std::vector<signed_distance> signed_distance_function;
	std::vector<signed_distance> signed_distance_function_temp;
	std::vector<std::string> file_paths, obj_names;
	KDTree* m_KDTree;
	KDTree* temp_KDTree;
	VecArray m_pts, temp_pts;
	float m1, m2, vol1, vol2, dens1, dens2, e, jr_mag, mk1, mk2;
	vec contact_p, contact_normal, U1, U2, Ur, r1, r2, omega1, omega2, jr;
	vec contact_p_th, contact_normal_th;
	int contact_index;
	Eigen::MatrixX3f  I1, I2;
	float y;
	std::vector<normal_point> normals;
	std::vector<normal_point> normals_temp;
	math::float3x4 final_transform1;
	math::float3x4 final_transform2;
	float m_last_update;
	float collision_time;
	bool speed_changed;
	bool found_collision;
	std::vector<vec> temp_vertices;
	std::vector<vec> model_vertices;
	std::vector<vec> vertices_non_dup;
	std::vector<vec> aabb_vertices;
	std::vector<vec> m_aabb_vertices;
	vec initial_pos_1, initial_pos_2;
	bool use_sdf;
	float U1x;
};

/**
 * Class Representing scene floor and background wall.
 */
class Grounds : public vvr::IRenderable
{
public:
	Grounds();
	Grounds(const float W, const float D, const float B, const float T, const vvr::Colour& colour);
	void draw() const override;

private:
	std::vector<math::Triangle> m_floor_tris;
	vvr::Colour m_col;
};

Grounds ground;