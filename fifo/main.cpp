// inserting into a vector
#include <iostream>
#include <vector>

/**
 * Definition
 * struct TreeNode {
 *  int val;
 *  TreeNode *left;
 *  TreeNode *right;
 *  TreeNode(int x): val(x), left(NULL), right(NULL) {}
 * };
 */
struct TreeNode {
  int val;
  TreeNode* left;
  TreeNode* right;
  TreeNode(int x) : val(x), left(NULL), right(NULL) {}
};

class Solution {
 private:
  /* data */
 public:
  Solution(/* args */){};
  ~Solution(){};
  std::pair<int, double> DFS(TreeNode* root) {
    if (!root) return {0,0};
    auto l = DFS(root->left);
    auto r = DFS(root->right);

    int a = l.first, c = r.first;
    double b = l.second, d = r.second;
    int tot = a + c + root->val;
    if((c-2*d <= a && a <= c) || (a-2*b <= c && c <= a)) {
      return {tot, (a+c)/2.0};
    }

    if(a-2*b > c) {
      return {tot, b+c};
    } else {
      return {tot, a+d};
    }
  }

  double minmalExecTime(TreeNode* root) {
    auto p = DFS(root);
    return p.first - p.second;
  }
};

int main() {
  std::vector<int> myvector(4, 100);
  std::vector<int>::iterator it;

  it = myvector.begin();
  it = myvector.insert(it, 200);

  myvector.insert(it, 300);

  myvector.pop_back();
  //   // "it" no longer valid, get a new one:
  //   it = myvector.begin();

  //   std::vector<int> anothervector (2,400);
  //   myvector.insert (it+2,anothervector.begin(),anothervector.end());

  //   int myarray [] = { 501,502,503 };
  //   myvector.insert (myvector.begin(), myarray, myarray+3);

  std::cout << "myvector contains:";
  for (it = myvector.begin(); it < myvector.end(); it++)
    std::cout << ' ' << *it;
  std::cout << '\n';

  return 0;
}