#include <type_traits>
#include <utility>
#include <vector>

template<typename valTp>
class Tree;

template<typename valTp>
class TreeNode
{
public:
  explicit TreeNode(const valTp& value = valTp(), TreeNode* parent = nullptr, TreeNode* child = nullptr) : _value(value), _parent(parent), _child(child) {};

  TreeNode(const TreeNode& t) = delete;
  
  TreeNode(TreeNode&& tNode)
  {
    _value = std::move(tNode._value);
    _parent = tNode._parent;
    _child = tNode._child;
  }

  TreeNode& operator=(TreeNode&& tNode)
  {
    _value = std::move(tNode._value);
    _parent = tNode._parent;
    _child = tNode._child;
  }

  void SetParent(TreeNode* parent) { _parent = parent; };
  void SetChild(TreeNode* child) { _child = child; };
  TreeNode* Parent() { return _parent; };
  TreeNode* Child() { return _child; };

  template<>
  friend class Tree<valTp>;
private:
  valTp _value;
  TreeNode* _parent;
  std::vector<TreeNode*> _child;
};

template<typename valTp>
class Tree
{
public:
  
private:
  TreeNode<valTp>* top;
};