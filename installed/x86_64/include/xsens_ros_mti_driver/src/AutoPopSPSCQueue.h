#include <boost/lockfree/spsc_queue.hpp>
template <typename T, size_t Capacity>
class AutoPopSPSCQueue {
 private:
  boost::lockfree::spsc_queue<T, boost::lockfree::capacity<Capacity>> queue;
 public:
  bool push_auto_pop(const T& item) {
    if (queue.push(item)) {
      return true;  // Item pushed successfully
    } else {
      // Queue is full, pop an item and then push
      T discarded_item;
      if (queue.pop(discarded_item)) {
        return queue.push(item);  // Should always succeed
      }
      return false;  // This should never happen in single-producer scenario
    }
  }

  bool pop(T& item) {
    return queue.pop(item);
  }

};
