#ifndef VOCABULARY_TREE_DATABASE_H
#define VOCABULARY_TREE_DATABASE_H

/// @todo Include some basic_types.h instead
#include <vocabulary_tree/vocabulary_tree.h>
#include <map>

namespace vt {

typedef uint32_t DocId;

/**
 * \brief Struct representing a single database match.
 *
 * \c score is in the range [0,2], where 0 is best and 2 is worst.
 */
struct Match
{
  DocId id;
  float score;

  Match() {}
  Match(DocId _id, float _score) : id(_id), score(_score) {}

  /// Allows sorting Matches in best-to-worst order with std::sort.
  bool operator<(const Match& other) const
  {
    return score < other.score;
  }
};

// Remove these, just make docs more confusing
typedef std::vector<Word> Document;
typedef std::vector<Match> Matches;

/**
 * \brief Class for efficiently matching a bag-of-words representation of a document (image) against
 * a database of known documents.
 */
class Database
{
public:
  /**
   * \brief Constructor
   *
   * If computing weights for a new vocabulary, \c num_words should be the size of the vocabulary.
   * If calling loadWeights(), it can be left zero.
   */
  Database(uint32_t num_words = 0);

  /**
   * \brief Insert a new document.
   *
   * \param document The set of quantized words in a document/image.
   * \return An ID representing the inserted document.
   */
  DocId insert(const std::vector<Word>& document);

  /**
   * \brief Find the top N matches in the database for the query document.
   *
   * \param      document The query document, a set of quantized words.
   * \param      N        The number of matches to return.
   * \param[out] matches  IDs and scores for the top N matching database documents.
   */
  void find(const std::vector<Word>& document, size_t N, std::vector<Match>& matches) const;

  /**
   * \brief Find the top N matches, then insert the query document.
   *
   * This is equivalent to calling find() followed by insert(), but may be more efficient.
   *
   * \param      document The document to match then insert, a set of quantized words.
   * \param      N        The number of matches to return.
   * \param[out] matches  IDs and scores for the top N matching database documents.
   */
  DocId findAndInsert(const std::vector<Word>& document, size_t N, std::vector<Match>& matches);

  /**
   * \brief Compute the TF-IDF weights of all the words. To be called after inserting a corpus of
   * training examples into the database.
   *
   * \param default_weight The default weight of a word that appears in none of the training documents.
   */
  void computeTfIdfWeights(float default_weight = 1.0f);

  /// Save the vocabulary word weights to a file.
  void saveWeights(const std::string& file) const;
  /// Load the vocabulary word weights from a file.
  void loadWeights(const std::string& file);

  // Save weights and documents
  //void save(const std::string& file) const;
  //void load(const std::string& file);

private:
  struct WordFrequency
  {
    DocId id;
    uint32_t count;

    WordFrequency(DocId _id, uint32_t _count) : id(_id), count(_count) {}
  };
  
  // Stored in increasing order by DocId
  typedef std::vector<WordFrequency> InvertedFile;

  /// @todo Use sorted vector?
  // typedef std::vector< std::pair<Word, float> > DocumentVector;
  typedef std::map<Word, float> DocumentVector;

  std::vector<InvertedFile> word_files_;
  std::vector<float> word_weights_;
  std::vector<DocumentVector> database_vectors_; // Precomputed for inserted documents

  void computeVector(const std::vector<Word>& document, DocumentVector& v) const;
  
  static void normalize(DocumentVector& v);
  static float sparseDistance(const DocumentVector& v1, const DocumentVector& v2);
};

} //namespace vt

#endif
