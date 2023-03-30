#include <boost/algorithm/minmax.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <iostream>
#include <omp.h>
#include <vector>

using namespace boost::numeric;

namespace clustering {
class DBSCAN {
public:
  typedef ublas::vector<float> FeaturesWeights;
  typedef ublas::matrix<float> ClusterData;
  typedef ublas::matrix<float> DistanceMatrix;
  typedef std::vector<uint32_t> Neighbors;
  typedef std::vector<int32_t> Labels;

  static ClusterData gen_cluster_data(size_t features_num, size_t elements_num);
  static FeaturesWeights std_weights(size_t s);

  DBSCAN(float eps, size_t min_elems, int num_threads = 0);
  DBSCAN();
  ~DBSCAN();

  void init(float eps, size_t min_elems, int num_threads = 0);
  void fit_precomputed(const DistanceMatrix &D);
  void wfit(const ClusterData &C, const FeaturesWeights &W);
  void reset();

  const Labels &get_labels() const;

private:
  inline void prepare_labels(size_t s);
  const DistanceMatrix calc_dist_matrix(const ClusterData &C,
                                        const FeaturesWeights &W);
  Neighbors find_neighbors(const DistanceMatrix &D, uint32_t pid);
  void dbscan(const DistanceMatrix &dm);

  float m_eps;
  size_t m_min_elems;
  int m_num_threads;
  float m_dmin;
  float m_dmax;

  Labels m_labels;
};

std::ostream &operator<<(std::ostream &o, DBSCAN &d);
} // namespace clustering
