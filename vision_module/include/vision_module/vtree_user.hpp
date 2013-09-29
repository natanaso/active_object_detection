#ifndef VTREE_USER_HPP_
#define VTREE_USER_HPP_

// Standard
#include <vector>

// Eigen
#include <Eigen/Core>

// Ros
#include <ros/ros.h>

//Vocabulary Tree includes
#include <vocabulary_tree/simple_kmeans.h>
#include <vocabulary_tree/vocabulary_tree.h>
#include <vocabulary_tree/database.h>
#include <vocabulary_tree/tree_builder.h>
#include <vocabulary_tree/simple_kmeans.h>

// FEATURE in {FPFH (1), PFH(2), VFH(3)}
#define FEATURE 1

class DocumentInfo
{
private:
	bool delete_document;
public:
	vt::Document* document;
	std::string name;
	
	// Constructors
	DocumentInfo()
		: delete_document(false)
	{}
	DocumentInfo(vt::Document* document, const std::string name)
		: delete_document(false), document(document), name(name)
	{}
	
	~DocumentInfo(){
		if (delete_document)
			delete document;
			//delete[] document;
	}
	
	void write(std::ostream& out);
	void read(std::istream& in);
};


class vtree_user
{
public:
#if FEATURE == 1
	typedef Eigen::Matrix<float, 1, 33> FeatureHist;
#elif FEATURE == 2
	typedef Eigen::Matrix<float, 1, 125> FeatureHist;
#elif FEATURE == 3
	typedef Eigen::Matrix<float, 1, 308> FeatureHist;
#else
	#error A valid feature definition is required!
#endif
	typedef std::vector<FeatureHist, Eigen::aligned_allocator<FeatureHist> > FeatureVector;
	
protected:
	bool DEBUG;
	
	// PARAMETERS for DATABASE
	std::string command, database_dir, clouds_dir;
	int tree_k, tree_levels;

	// DATABASE
	std::vector<vt::Document> docs;				// build_database()
	vt::TreeBuilder<FeatureHist> tree_builder;	// build_database()
	vt::VocabularyTree<FeatureHist> tree;		// build_database()
	vt::Database* db;							// build_database()
	std::vector<std::string> cloud_names;		// Populated by trace_directory()
	std::map<int, DocumentInfo*> documents_map; // build_database()
	

	// feature and keypoint computation
	double keypoint_radius_, normal_radius_, feature_radius_;
		
	// CLUSTERING PARAMETERS
	int enable_clustering, min_cluster_size;
	double radius_adaptation_r_min, radius_adaptation_r_max, radius_adaptation_K, radius_adaptation_A;
		
private:	
	/**
	 * \brief Keypoints extension class
	 
	template <class F>
	class KeypointExt 
	{
	public:
		F keypoint;
		vt::Word word;
		unsigned int cluster;
	  
		KeypointExt(F keypoint, vt::Word word, unsigned int cluster = 0)
			: keypoint(keypoint), word(word), cluster(cluster)
		{}
	};
	*/
	
public:
	vtree_user( const std::string command, const std::string database_dir,
				const std::string clouds_dir, int tree_k = 16, int tree_levels = 5 );
	~vtree_user();
	
	int start();
	
	// Scoring
	std::pair<float,std::string> top_match( const Eigen::Matrix<float,4,Eigen::Dynamic>& cloud_matrix_map );

	void match_list( const Eigen::Matrix<float,4,Eigen::Dynamic>& cloud_matrix_map,
					 std::vector<std::pair<float,std::string> > & match_names,
					 int num_match );
					 	
	/*
	void match_list( const Eigen::Matrix<float,3,Eigen::Dynamic>& cloud_matrix_map,
					 std::vector<std::pair<float,std::string> > & match_names,
					 std::vector<std::pair<float,std::string> > & cluster_match_names,
					 int num_match );
	*/			 	
	// Mutators
	void set_command( const std::string command ){
		this->command = command;
	}
	
	void set_database_dir( const std::string database_dir ){
		this->database_dir = database_dir;
	}
	
	void set_clouds_dir( const std::string clouds_dir ){
		this->clouds_dir = clouds_dir;
	}
	
	
private:

	// Database
	void build_database(const std::string directory);
	void save_database(const std::string directory);
	void save_database_without_tree(const std::string directory);
	void load_database(const std::string directory);
	void add_pointcloud_to_database(vt::Document& doc, const std::string name);
	
	void rectify_histogram( float *histogram );
			
	void trace_directory(const char* dir, const char* prefix, std::vector<FeatureVector>& clouds, bool onlySaveClouds = false);
	
	void process_file(std::string& filename, std::vector<FeatureVector>& clouds, bool onlySaveClouds = false);
	
	void compute_features( const Eigen::Matrix<float,4,Eigen::Dynamic>& cloud_matrix_map,
								  vt::Document &full_doc );
								  
	
	void compute_features( const std::string& cloud_filename,
								  FeatureVector &feature_vector );

};

class ros_vtree_user : public vtree_user
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	
protected:
	virtual ros::NodeHandle& getNodeHandle() { return nh_; }
  	virtual ros::NodeHandle& getPrivateNodeHandle() { return private_nh_; }

public:
	ros_vtree_user( ros::NodeHandle & nh );
	
};

#endif
