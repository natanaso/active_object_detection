/*
 * Copyright (c) 2011, University of Pennsylvania
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \author Bharath Sankaran
 *
 * @b comprises of main declarations for nbv tree class
 */
// Adding guards
#ifndef NBV_TREE_H_
#define NBV_TREE_H_

#include "vision_module/common.h"
#include <boost/multi_array.hpp>
using namespace nbv_tree;

//typedef Eigen::Matrix<float, 1, 308> Feature;
typedef Eigen::Matrix<float, 1, 125> Feature;
typedef std::vector<Feature, Eigen::aligned_allocator<Feature> > FeatureVector;

const int COLORS = 13;

class DocumentInfo
{
private:
	bool delete_document;
public:
	vt::Document* document;
	std::string name;
	DocumentInfo();
	DocumentInfo(vt::Document* document, std::string& name);
	~DocumentInfo();
	void write (std::ostream& out);
	void read(std::istream& in);
};

class NBVTree
{

protected:
	ros::NodeHandle nh_;
public:
	// LOGGING
	int frame_number;
	std::map<std::string, int> stat_summary_map;
	bool extract_roi_,detect_objects_;

	// VISUALIZATION
	// think about visualization later
	std::vector<unsigned int> cluster_sizes;
	std::string output_cloud_topic_;

	// DATABASE
	std::vector<vt::Document> docs;
	vt::TreeBuilder<Feature> tree_builder;
	vt::VocabularyTree<Feature> tree;
	vt::Database* db;
	std::vector<std::string> cloud_names;
	std::map<int, DocumentInfo*> documents_map;

	// RECOGNITION
	size_t pc_keypoints_count;
	PointCloud::Ptr keypoints_;

	//OBSERVATION MODEL
	bool get_match,obj_present,multi_hypothesis;
	std::string match_name,best_name;
    double best_score;
    std::vector<double> score_vector;
    int hypotheses_no_,views_no_,yaw_;
    MatrixXf view_correspondence_;

    // CONFUSION MATRIX
    bool enable_confusion;


	//map of DocumentIDs and their scores in the database
	std::map<uint32_t, float> matches_map;
	std::list<int> sliding_window;
	std::map<int, int> last_templates;

	// Planning parameters
	int num_sta,num_hid,num_act,num_obs;

	// PARAMETERS
	std::string command, database_location, clouds_directory, base_dir_,base_name_,
	viewpoint_file, correspondence_file;
	int votes_count, tree_k, tree_levels, min_cluster_size, object_id;
	double unknown_object_threshold;
	int enable_clustering, enable_incremental_learning, sliding_window_size;
	double radius_adaptation_r_min, radius_adaptation_r_max, radius_adaptation_K, radius_adaptation_A;
	bool DEBUG;



public:
	NBVTree();

	// overloading the constructor for smart sensor class
	NBVTree(ros::NodeHandle & nh);

	~NBVTree();
  
  /** \brief Create a visualization window and start the visualization threas
   * \param enable_visualization
   */
  std::string process_pointcloud(PointCloud::Ptr point_cloud);
  
  //std::string process_pointcloud(PointCloud::Ptr point_cloud,std::string filename);

  //std::string process_pointcloud(PointCloud::Ptr point_cloud_in,std::vector<double> &scores);

  bool process_pointcloud_simple( PointCloud::Ptr point_cloud_in, int &hypothesis, int &viewpoint);
  
  int start();

  /** \brief builds the vocabulary tree and trains the database 
   * \param directory directory with training images
   */
	void build_database(std::string directory);

  /** \brief only called with "sift_only" command argument
      for extracting of keypoints
   * \param directory directory with training images
   */
	void process_pointclouds(std::string directory);

  /** \brief saves images.weights, images.documents (faster than save_database function)
   * \param directory target directory for database
   */  
	void save_database_without_tree(std::string& directory);

  /** \brief saves images.tree, images.weights, images.documents
   * \param directory target directory for database
   */
	void save_database(std::string& directory);

  /** \brief loads the database, that is images.tree, images.weights, images.documents
   * \param directory storage location for database
   */
	void load_database(std::string& directory);
  
  /** \brief adds new templates to the database
   * \param doc full database document
   * \param name new template name (object + time stamp in this case)
   */
	void add_pointcloud_to_database(vt::Document& doc, std::string& name);
  
	//void get_cluster_surface(PointCloud::Ptr cloud, Eigen::Vector3d &centroid, pcl::PointIndices::Ptr& return_indices,PointCloud::Ptr & cloud_cluster_new);

	//void get_surface(Eigen::Vector3d viewpoint, Eigen::Vector4f centroid,std::string ply_name, PointCloud::Ptr &surface);

	void project_centroid(PointCloud::Ptr input_cloud, std::vector<Eigen::Vector4f>& centroids);
	void project_centroid(PointCloud::Ptr input_cloud, std::vector<Eigen::Vector4f>& centroids, std::vector<pcl::PointIndices>& cluster_indices);

	//void get_hypotheses(PointCloud::Ptr cloud, int view_point, MatrixXd &scores);
	//void get_hypotheses(PointCloud::Ptr cloud, Eigen::Vector3d view_point, std::vector<double> &scores);

	//void compute_best_score(PointCloud::Ptr cloud_new, double &score, std::string &name,Eigen::Vector4f centroid);

	void pose_refinement(PointCloud::Ptr input_cloud, int viewpoint, int hypothesis, Eigen::Matrix3f &transform);
	void pose_refinement(PointCloud::Ptr input_cloud, Eigen::Matrix3f& rotation);
	void align_clouds(PointCloud::Ptr& input_cloud, PointCloud::Ptr target_cloud, PointCloud::Ptr& final_cloud);

	void get_cluster_centroid(PointCloud::Ptr cloud, Eigen::Vector3d &centroid, std::vector<Eigen::Vector4f>& centroids, std::vector<pcl::PointIndices>& cluster_indices, PointCloud::Ptr& cloud_cluster_new);

	// Functions to obtain the score from the vocabulary tree for a query pointcloud
	int get_vp_score( PointCloud::Ptr point_cloud_in );
	int get_vp_score( std::string top_match, std::string query );
	std::string top_match_name( PointCloud::Ptr point_cloud_in );
	void match_list( PointCloud::Ptr point_cloud_in, 
					 std::vector<std::pair<float,std::string> > & match_names );
	
protected:
  /** \brief recursively traces the directory with images
   * \param dir parent directory
   * \param prefix 
   * \param images vector of extracted keypoints
   * \param onlySaveImages whether we only extract keypoints and save images with them 
   */
	void trace_directory(const char* dir, const char* prefix, std::vector<FeatureVector>& clouds, bool onlySaveClouds = false);
 
 /** \brief Accumulates the scores from the clusters and updates the map of matches 
  * (matches_map) which is a map of Document IDs  and their scores in the database. 
  * \param matches - vector of Match-es (see vocabulary tree API)
  * \param size - currently unused
  */
	void update_matches_map(vt::Matches& matches, size_t size);

  /** \brief extract keypoints from training images and optionally saves them
   * \param filename input training image
   * \param images extracted keypoints
   * \param onlySaveClouds whether to save clouds or not
   */
	void process_file(std::string& filename, std::vector<FeatureVector>& clouds, bool onlySaveClouds = false);

  /** \brief extracts PFH keypoints
   * \param filename input image
   * \param frames_only currently redundant @TODO: remove??
   */
	std::vector<Keypoint_p> extract_keypoints(PointCloud::Ptr point_cloud, bool frames_only = false);

	/** \brief extracts NARF Keypoints
	   * \param filename input image
	   * \param frames_only currently redundant @TODO: remove??
	   */
	void compute_keypoints (PointCloud::Ptr &points, PointCloud::Ptr &keypoints);

	//double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
	//void compute_policy(bool &status, int &best_act, int &init_sta, sarsop_user &su,std::vector<double> init_bel);

	//void get_init_state(int &init_sta);

	//void init_views(MatrixXf &views);

	void load_matrix(MatrixXf & values ,std::string filename,int row, int col);
	void load_matrix(boost::multi_array<double,3>  & values ,std::string filename,int row, int col, int third);

	//void get_surface(Eigen::Vector4f viewpoint, Eigen::Vector4f centroid,std::string ply_name, PointCloud::Ptr &surface);
	//void nearest_viewpoint_id(Eigen::Vector3d search_point, int &viewpoint);

};

#endif
