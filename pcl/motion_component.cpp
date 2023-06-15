

// https://blog.csdn.net/qq_32761549/article/details/120738568


#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>

using namespace std;

typedef struct binaryNode {
  int value;
  struct binaryNode* left;
  struct binaryNode* right;
}* pNode;

class MyClass {
 public:
  int num;
  string name;
};

void creatBinary(pNode& T) {
  int _v = 10;
  T = new binaryNode;
  T->value = _v;
  creatBinary(T->left);
  creatBinary(T->right);
}

Eigen::Vector3d transformOfstart(Eigen::Vector3d* pi) {
  // interpolation ratio
  double s;
  double t_start, t_end, t_curr;
  // q_last_curr t_last_curr 是结束时刻坐标系转到起始时刻坐标系 的 旋转 和 平移
  Eigen::Quaterniond q_last_curr;
  Eigen::Vector3d t_last_curr;

  s = t_curr / (t_end - t_start);

  Eigen::Quaterniond q_point_last =
      Eigen::Quaterniond::Identity().slerp(s, q_last_curr);  // 求姿态的球面插值

  Eigen::Vector3d t_point_last = s * t_last_curr;  // 求位移的线性插值
  Eigen::Vector3d point(pi->x(), pi->y(), pi->z());  // 把当前点的坐标取出
  Eigen::Vector3d un_point =
      q_point_last * point +
      t_point_last;  // 通过旋转和平移将 当前点转到帧起始时刻坐标系下的坐标

  return un_point;
}
int main(int argc, char** argv) {
  MyClass m;
  MyClass* p = new MyClass();

  return 0;
}