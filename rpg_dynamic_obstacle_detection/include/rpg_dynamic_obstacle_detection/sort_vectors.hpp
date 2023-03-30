#pragma once

namespace rpg_dynamic_obstacle_detection {
template <typename T, typename Compare>
void getSortPermutation(std::vector<unsigned> &out, const std::vector<T> &v,
                        Compare compare = std::less<T>()) {
  out.resize(v.size());
  std::iota(out.begin(), out.end(), 0);

  std::sort(out.begin(), out.end(),
            [&](unsigned i, unsigned j) { return compare(v[i], v[j]); });
}

template <typename T>
void applyPermutation(const std::vector<unsigned> &order, std::vector<T> &t) {
  assert(order.size() == t.size());
  std::vector<T> st(t.size());
  for (unsigned i = 0; i < t.size(); i++) {
    st[i] = t[order[i]];
  }
  t = st;
}

template <typename T, typename... S>
void applyPermutation(const std::vector<unsigned> &order, std::vector<T> &t,
                      std::vector<S> &... s) {
  applyPermutation(order, t);
  applyPermutation(order, s...);
}

template <typename T, typename Compare, typename... SS>
void sortVectors(const std::vector<T> &t, Compare comp,
                 std::vector<SS> &... ss) {
  std::vector<unsigned> order;
  getSortPermutation(order, t, comp);
  applyPermutation(order, ss...);
}

// make less verbose for the usual ascending order
template <typename T, typename... SS>
void sortVectorsAscending(const std::vector<T> &t, std::vector<SS> &... ss) {
  sortVectors(t, std::less<T>(), ss...);
}

template <typename T, typename... SS>
void sortVectorsDescending(const std::vector<T> &t, std::vector<SS> &... ss) {
  sortVectors(t, std::greater<T>(), ss...);
}

template <class RAIter, class Compare>
void argsort(RAIter iterBegin, RAIter iterEnd, Compare comp,
             std::vector<size_t> &indexes) {

  std::vector<std::pair<size_t, RAIter>> pv;
  pv.reserve(iterEnd - iterBegin);

  RAIter iter;
  size_t k;
  for (iter = iterBegin, k = 0; iter != iterEnd; iter++, k++) {
    pv.push_back(std::pair<int, RAIter>(k, iter));
  }

  std::sort(pv.begin(), pv.end(),
            [&comp](const std::pair<size_t, RAIter> &a,
                    const std::pair<size_t, RAIter> &b) -> bool {
              return comp(*a.second, *b.second);
            });

  indexes.resize(pv.size());
  std::transform(
      pv.begin(), pv.end(), indexes.begin(),
      [](const std::pair<size_t, RAIter> &a) -> size_t { return a.first; });
}
} // namespace rpg_dynamic_obstacle_detection
