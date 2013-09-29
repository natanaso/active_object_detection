#ifndef VOCABULARY_TREE_VOCABULARY_TREE_H
#define VOCABULARY_TREE_VOCABULARY_TREE_H

#include "vocabulary_tree/distance.h"
#include "vocabulary_tree/feature_allocator.h"
#include <stdint.h>
#include <vector>
#include <cassert>
#include <limits>
#include <fstream>
#include <stdexcept>
#include <boost/format.hpp>

namespace vt {

typedef int32_t Word;

/**
 * \brief Optimized vocabulary tree quantizer, templated on feature type and distance metric
 * for maximum efficiency.
 *
 * \c Feature is the data type of one feature. It has no requirements except compatibility with the distance metric.
 *
 * \c Distance is a functor that computes the distance between two Feature objects. It must have a \c result_type
 * typedef specifying the type of the returned distance. For the purposes of VocabularyTree, this need not even be
 * a metric; distances simply need to be comparable.
 *
 * \c FeatureAllocator is an STL-compatible allocator used to allocate Features internally.
 */
template<class Feature, class Distance = distance::L2<Feature>,
         class FeatureAllocator = typename DefaultAllocator<Feature>::type>
class VocabularyTree
{
public:
  /**
   * \brief Constructor, empty tree.
   *
   * \param d Functor for computing the distance between two features
   * 
   * @todo Allocator parameter, also in MutableVocabularyTree, TreeBuilder...
   */
  VocabularyTree(Distance d = Distance());

  /**
   * \brief Constructor, loads vocabulary from file.
   *
   * \param file Saved vocabulary file
   * \param d    Functor for computing the distance between two features
   */
  VocabularyTree(const std::string& file, Distance d = Distance());

  /// Quantizes a feature into a discrete word.
  Word quantize(const Feature& f) const;

  /// Get the depth (number of levels) of the tree.
  uint32_t levels() const;
  /// Get the branching factor (max splits at each node) of the tree.
  uint32_t splits() const;
  /// Get the number of words the tree contains.
  uint32_t words() const;

  /// Clears vocabulary, leaving an empty tree.
  void clear();

  /// Save vocabulary to a file.
  void save(const std::string& file) const;
  /// Load vocabulary from a file.
  void load(const std::string& file);

protected:
  typedef typename Distance::result_type distance_type;
  
  std::vector<Feature, FeatureAllocator> centers_;
  std::vector<uint8_t> valid_centers_; /// @todo Consider bit-vector
  Distance distance_;

  uint32_t k_; // splits, or branching factor
  uint32_t levels_;
  uint32_t num_words_; // number of leaf nodes
  uint32_t word_start_; // number of non-leaf nodes, or offset to the first leaf node

  bool initialized() const { return num_words_ != 0; }

  void setNodeCounts();
};


template<class Feature, class Distance, class FeatureAllocator>
VocabularyTree<Feature, Distance, FeatureAllocator>::VocabularyTree(Distance d)
  : distance_(d), k_(0), levels_(0), num_words_(0), word_start_(0)
{
}

template<class Feature, class Distance, class FeatureAllocator>
VocabularyTree<Feature, Distance, FeatureAllocator>::VocabularyTree(const std::string& file, Distance d)
  : distance_(d), k_(0), levels_(0), num_words_(0), word_start_(0)
{
  load(file);
}

template<class Feature, class Distance, class FeatureAllocator>
Word VocabularyTree<Feature, Distance, FeatureAllocator>::quantize(const Feature& f) const
{
  assert( initialized() );

  int32_t index = -1; // virtual "root" index, which has no associated center.
  for (unsigned level = 0; level < levels_; ++level) {
    // Calculate the offset to the first child of the current index.
    int32_t first_child = (index + 1) * splits();
    // Find the child center closest to the query.
    int32_t best_child = first_child;
    distance_type best_distance = std::numeric_limits<distance_type>::max();
    for (int32_t child = first_child; child < first_child + (int32_t)splits(); ++child) {
      if (!valid_centers_[child])
        break; // Fewer than splits() children.
      distance_type child_distance = distance_(f, centers_[child]);
      if (child_distance < best_distance) {
        best_child = child;
        best_distance = child_distance;
      }
    }
    index = best_child;
  }

  return index - word_start_;
};

template<class Feature, class Distance, class FeatureAllocator>
uint32_t VocabularyTree<Feature, Distance, FeatureAllocator>::levels() const
{
  return levels_;
}

template<class Feature, class Distance, class FeatureAllocator>
uint32_t VocabularyTree<Feature, Distance, FeatureAllocator>::splits() const
{
  return k_;
}

template<class Feature, class Distance, class FeatureAllocator>
uint32_t VocabularyTree<Feature, Distance, FeatureAllocator>::words() const
{
  return num_words_;
}

template<class Feature, class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::clear()
{
  centers_.clear();
  valid_centers_.clear();
  k_ = levels_ = num_words_ = word_start_ = 0;
}

template<class Feature, class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::save(const std::string& file) const
{
  /// @todo Support serializing of non-"simple" feature classes
  /// @todo Some identifying name for the distance used
  assert( initialized() );

  std::ofstream out(file.c_str(), std::ios_base::binary);
  out.write((char*)(&k_), sizeof(uint32_t));
  out.write((char*)(&levels_), sizeof(uint32_t));
  uint32_t size = centers_.size();
  out.write((char*)(&size), sizeof(uint32_t));
  out.write((char*)(&centers_[0]), centers_.size() * sizeof(Feature));
  out.write((char*)(&valid_centers_[0]), valid_centers_.size());
}

template<class Feature, class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::load(const std::string& file)
{
  clear();

  std::ifstream in;
  in.exceptions(std::ifstream::eofbit | std::ifstream::failbit | std::ifstream::badbit);

  uint32_t size;  
  try {
    in.open(file.c_str(), std::ios_base::binary);
    in.read((char*)(&k_), sizeof(uint32_t));
    in.read((char*)(&levels_), sizeof(uint32_t));
    in.read((char*)(&size), sizeof(uint32_t));
    centers_.resize(size);
    valid_centers_.resize(size);
    in.read((char*)(&centers_[0]), centers_.size() * sizeof(Feature));
    in.read((char*)(&valid_centers_[0]), valid_centers_.size());
  }
  catch (std::ifstream::failure& e) {
    throw std::runtime_error( (boost::format("Failed to load vocabulary tree file '%s'") % file).str() );
  }

  setNodeCounts();
  assert(size == num_words_ + word_start_);
}

template<class Feature, class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::setNodeCounts()
{
  num_words_ = k_;
  word_start_ = 0;
  for (uint32_t i = 0; i < levels_ - 1; ++i) {
    word_start_ += num_words_;
    num_words_ *= k_;
  }
}

} //namespace vt

#endif
