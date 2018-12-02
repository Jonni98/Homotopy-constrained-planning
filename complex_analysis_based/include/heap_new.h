
#include <vector>
#include <iostream>
#include <memory>

namespace ad_planning_utilities
{

/*!
 * \class Heap
 */
template <typename KeyType>
class Heap
{
public:
  /*!
   * \struct Element
   * \brief The base class for any element that is stored in the heap
   */
  struct Element
  {
    Element(KeyType key = KeyType(), int heap_index = -1)
      : key_(key), heap_index_(heap_index)
    {
    }

    KeyType key_;
    int heap_index_;
  };

  typedef std::shared_ptr<Element> Ptr;

  Heap()
  {
  }

  Heap(int initial_size)
  {
    data_.reserve(initial_size);
  }

  ~Heap()
  {
    clear();
  }

  /*!
   * \brief Builds a min-heap from the given vector of data
   * \param data
   */
  void buildHeap(const std::vector<Ptr> &data)
  {
    clear();
    for (int k = 0; k < static_cast<int>(data.size()); ++k)
    {
      data[k]->heap_index_ = k;
      data_.push_back(data[k]);
    }

    for (int i = size() / 2; i >= 0; --i)
    {
      heapifyDown(i);
    }
  }

  /*!
   * \brief Removes the minimum-priority element in the heap in O(lg n) time
   * \return
   */
  Ptr remove()
  {
    // Remove the first element in the heap and replace it with the last
    Ptr temp = data_[0];
    data_[0] = data_[size() - 1];
    data_[0]->heap_index_ = 0;
    data_.pop_back();

    // Restore the min-heap property
    heapifyDown(0);

    temp->heap_index_ = -1;
    return temp;
  }

  /*!
   * \brief Removes a specific element from the heap
   * \param e
   */
  void remove(Ptr e)
  {
    const int heap_index = e->heap_index_;
    e->heap_index_ = -1;

    if (data_.empty())
    {
      std::cerr << "Requesting to remove element from an empty heap!" << std::endl;
      return;
    }
    else if (data_.size() == 1)
    {
      std::cout << "Removing last element from heap!" << std::endl;
      data_.pop_back();
      return;
    }

    // Swap the last element in the heap with this element, and remove the last element
    data_[heap_index] = data_[size() - 1];
    data_[heap_index]->heap_index_ = heap_index;
    data_.pop_back();

    // Restore the min-heap property by percolating up or percolating down
    int parent_index = getParent(heap_index);
    if (parent_index < 0 // Element at index is the root of the tree
        || data_[parent_index]->key_ < data_[heap_index]->key_)
      heapifyDown(heap_index);
    else
      heapifyUp(heap_index);
  }

  /*!
   * \brief top
   * \return
   */
  Ptr top()
  {
    if (data_.empty())
    {
      std::cerr << "Error in top: Heap is empty!" << std::endl;
      return nullptr;
    }

    return data_[0];
  }

  /*!
   * \brief Decrease the key of a given heap element in O(lg n) time
   * \param e The heap element
   * \param k The new key of the element
   * \return
   */
  bool decreaseKey(Ptr e, KeyType k)
  {
    // Get the index of the element
    int i = e->heap_index_;

    if (i == -1)
    {
      std::cerr << "Error in decreaseKey: Invalid heap index for heap element" << std::endl;
      return false;
    }

    if (k > data_[i]->key_)
    {
      std::cerr << "Error in decreaseKey: New key must be smaller than existing key" << std::endl;
      return false;
    }

    data_[i]->key_ = k;

    while (i > 0 && data_[getParent(i)]->key_ > data_[i]->key_)
    {
      // Perform a swap of data_[i] with data_[getParent(i)]
      Ptr temp = data_[i];
      data_[i] = data_[getParent(i)];
      data_[i]->heap_index_ = i;
      data_[getParent(i)] = temp;
      data_[getParent(i)]->heap_index_ = getParent(i);

      i = getParent(i);
    }

    return true;
  }

  /*!
   * \brief Insert the element into the heap in O(lg n) time
   * \param e
   */
  void insert(Ptr e)
  {
    e->heap_index_ = static_cast<int>(data_.size());
    data_.push_back(e);

    // Percolate up
    int i = size() - 1;
    int parent = getParent(i);
    while (parent != -1 && data_[i]->key_ < data_[parent]->key_)
    {
      // Perform a swap
      Ptr temp = data_[i];
      data_[i] = data_[parent];
      data_[i]->heap_index_ = i;
      data_[parent] = temp;
      data_[parent]->heap_index_ = parent;
      // Continue to percolate up the tree
      i = parent;
      parent = getParent(parent);
    }
  }

  /*!
   * \return Returns the number of elements currently in the heap
   */
  int size() const
  {
    return static_cast<int>(data_.size());
  }

  /*!
   * \brief Clear all elements from the heap
   */
  void clear()
  {
    for (Ptr e : data_)
    {
      if (e)
        e->heap_index_ = -1;
    }

    data_.clear();
  }

  const std::vector<Ptr > &getData() const
  {
    return data_;
  }

private:
  int getParent(int i) const
  {
    return (i == 0 ? -1 : (i - 1) / 2);
  }

  int getLeftChild(int i) const
  {
    return ((2 * i + 1) < size() ? (2 * i + 1) : -1);
  }

  int getRightChild(int i) const
  {
    return ((2 * i + 2) < size() ? (2 * i + 2) : -1);
  }

  /*!
   * \brief Restore the heap property in O(lg n) time by percolating down from the specified element
   * \param i
   */
  void heapifyDown(int i)
  {
    int l = getLeftChild(i);
    int r = getRightChild(i);
    int root;

    if (l != -1 && data_[l]->key_ < data_[i]->key_)
    {
      root = l;
    }
    else root = i;

    if (r != -1 && data_[r]->key_ < data_[root]->key_)
    {
      root = r;
    }

    if (root != i)
    {
      // Swap data_[i] with data_[root]
      Ptr temp = data_[i];
      data_[i] = data_[root];
      data_[i]->heap_index_ = i;
      data_[root] = temp;
      data_[root]->heap_index_ = root;

      heapifyDown(root);
    }
  }

  /*!
   * \brief Restore the heap property in O(lg n) time by percolating up from the specified element
   * \param i
   */
  void heapifyUp(int i)
  {
    int parent = getParent(i);
    if (parent != -1 && data_[i]->key_ < data_[parent]->key_)
    {
      // Perform a swap
      Ptr temp = data_[i];
      data_[i] = data_[parent];
      data_[i]->heap_index_ = i;
      data_[parent] = temp;
      data_[parent]->heap_index_ = parent;

      // Continue to percolate up the tree
      heapifyUp(parent);
    }
  }

  std::vector<Ptr> data_;

};

} // namespace ad_planning_utilities

