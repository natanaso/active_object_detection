// Standard
#include <dirent.h>		// Directory tracing
#include <sys/stat.h>	// Directory tracing

// Pcl
#include <pcl16/point_types.h>
#include <pcl16/point_cloud.h>

#include <pcl16/features/fpfh.h>
#include <pcl16/features/vfh.h>
#include <pcl16/features/pfh.h>
#include <pcl16/io/pcd_io.h>							// Load pcd files
#include <pcl16/features/normal_3d.h>				// Normal estimation
#include <pcl16/keypoints/uniform_sampling.h>	// Keypoint extraction

// Ros
#include <ros/console.h>

// Custom
#include "vision_module/vtree_user.hpp"
//#include "pcd_utils.hpp"

	
void DocumentInfo::write(std::ostream& out) 
{
	size_t length = name.length();
	out.write((char*) &length, sizeof(size_t));
	out.write(name.c_str(), name.length());
	size_t doc_length = document->size();
	out.write((char*) &doc_length, sizeof(size_t));
	out.write((char*) &(document->at(0)), doc_length * sizeof(vt::Word));
}

void DocumentInfo::read(std::istream& in) 
{
	size_t length;
	in.read((char*) &length, sizeof(size_t));
	char* name = new char[length + 1];
	in.read(name, length);
	name[length] = 0;
	this->name.assign(name);  
	size_t doc_length;
	in.read((char*) &doc_length, sizeof(size_t));
	document = new vt::Document(doc_length);
	in.read((char*) &document->at(0), doc_length * sizeof(vt::Word));
	this->delete_document = true;
	delete[] name;
}
/////////////////////////////////////////

vtree_user::vtree_user( const std::string command, const std::string database_dir, 
						const std::string clouds_dir, int tree_k /*= 16*/, int tree_levels /*= 5*/ )
	: DEBUG(false), 
	  command(command), database_dir(database_dir), clouds_dir(clouds_dir),
	  tree_k(tree_k), tree_levels(tree_levels), tree_builder(FeatureHist::Zero()), 
	  
	  // Keypoint and feature parameters
	  keypoint_radius_(0.02), normal_radius_(0.04), feature_radius_(0.063),

	  // Clustering parameters
	  enable_clustering(0), min_cluster_size(30), 
	  radius_adaptation_r_min(200.0), radius_adaptation_r_max(600.9),
	  radius_adaptation_K(0.02), radius_adaptation_A(800.0)
	  
	  // Other Parameters
	  //object_id(700000), unknown_object_threshold(0.3),
	  //enable_incremental_learning(0), sliding_window_size(10)
{

}


vtree_user::~vtree_user()
{
	ROS_WARN("DESTRUCTING TREE");
	delete db;

	for (std::map<int, DocumentInfo*>::iterator iter = documents_map.begin(); 
			iter != documents_map.end(); ++iter)
		delete iter->second;
		
	ROS_WARN("TREE DESTRUCTED");
}

////////////////////////////////////////

// global variables used: clouds_dir, database_dir
int 
vtree_user::start()
{
	//if init build and save the database
	if (command.compare("/init") == 0)
	{
		build_database(clouds_dir);
		save_database(database_dir);
	}
	//load previously built database and perform recognition
	else if (command.compare("/load") == 0)
		load_database(database_dir);

	//only extract features and save them in a specified images_directory
	else if (command.compare("/features_only") == 0)
	{
		std::vector<FeatureVector> clouds;
		trace_directory(clouds_dir.c_str(), "", clouds, true);	// TODO: What does this do?
	}
	else
		return 1;
	
	return 0;
}

// global variables used: 
// tree_builder, tree, docs, db, documents_map, cloud_names
// tree_k, tree_levels
void
vtree_user::build_database(const std::string directory)
{

	std::vector<FeatureVector> cloud_features_vec;	// vector of vectors of feature histograms, a set of features for each cloud
	trace_directory(directory.c_str(), "", cloud_features_vec);	// trace the directory of cloud files and process each file
	
	
	ROS_INFO("Preparing features for the tree...");
	FeatureVector all_features;
	size_t num_feat = 0;
	for( std::vector<FeatureVector>::iterator iter = cloud_features_vec.begin();
		 iter != cloud_features_vec.end(); ++iter)
		 num_feat += iter->size();
	
	all_features.reserve( num_feat );
	for( std::vector<FeatureVector>::iterator iter = cloud_features_vec.begin();
		 iter != cloud_features_vec.end(); ++iter)	 
		 all_features.insert( all_features.end(), iter->begin(), iter->end() );
		 
  
	ROS_INFO_STREAM("Building a tree with " << all_features.size() << " nodes...");
	tree_builder.build(all_features, tree_k, tree_levels);
	tree = tree_builder.tree();
	
	ROS_INFO("Creating the documents...");
	docs.resize(cloud_features_vec.size());
	
	for (long unsigned int i = 0; i < cloud_features_vec.size(); ++i)
		for (long unsigned int j = 0; j < cloud_features_vec[i].size(); ++j)
			docs[i].push_back(tree.quantize(cloud_features_vec[i][j]));
  
	ROS_INFO("Creating database...");
	db = new vt::Database(tree.words());

	ROS_INFO("Populating the database with the documents...");
	for (long unsigned int i = 0; i < cloud_features_vec.size(); ++i)
		documents_map[db->insert(docs[i])] = new DocumentInfo(&(docs[i]),cloud_names[i]);

	ROS_INFO("Training database...");
	db->computeTfIdfWeights(1);
	
	ROS_INFO("Database created!");
}

// Global variables used: tree
void
vtree_user::save_database( const std::string directory )
{
	ROS_INFO("Saving the tree...");
	std::string tree_file(directory + "/clouds.tree");
	tree.save(tree_file.c_str());
	save_database_without_tree(directory);
}

// Global variables used: documents_map, db
void
vtree_user::save_database_without_tree( const std::string directory)
{
	ROS_INFO("Saving documents...");
	std::string documents_file(directory + "/clouds.documents");

	std::ofstream out(documents_file.c_str(), std::ios::out | std::ios::binary);
	size_t map_size = documents_map.size();

	out.write((char*) &map_size, sizeof(size_t));
	std::map<int, DocumentInfo*>::iterator iter;

	for (iter = documents_map.begin(); iter != documents_map.end(); ++iter)
	{
		out.write((char*) &iter->first, sizeof(int));
		iter->second->write(out);
	}

	ROS_INFO("Saving weights...");
	std::string weights_file(directory + "/clouds.weights");
	db->saveWeights(weights_file.c_str());
	
	out.close();
	ROS_INFO("Done! Press Ctrl+C");
}

// Global variables used: documents_map, db
void 
vtree_user::load_database(const std::string directory)
{
	ROS_INFO("Loading the tree...");
	std::string tree_file(directory + "/clouds.tree");
	tree.load(tree_file.c_str());

	ROS_INFO("Initializing the database...");
	db = new vt::Database(tree.words());
	std::string documents_file(directory + "/clouds.documents");
	ROS_INFO("Loading the documents... (%s)", documents_file.c_str());

	std::ifstream in(documents_file.c_str(), std::ios::in | std::ios::binary);
	size_t map_size;
	in.read((char*) &map_size, sizeof(size_t));

	for (size_t i = 0; i < map_size; ++i)
	{
		int id;
		DocumentInfo* document_info = new DocumentInfo();
		in.read((char*) &id, sizeof(int));
		document_info->read(in);
		vt::Document* doc = document_info->document;
		int d = db->insert(*doc);
		documents_map[d] = document_info;
	}

	ROS_INFO("Loading weights...");
	std::string weights_file(directory + "/clouds.weights");
	db->loadWeights(weights_file.c_str());
	
	in.close();
	ROS_INFO("READY!");
}

// Global variables used: docs, database_dir
void
vtree_user::add_pointcloud_to_database(vt::Document& doc, const std::string name)
{
	docs.push_back(doc);
	documents_map[db->insert(doc)] = new DocumentInfo(&doc, name);
	db->computeTfIdfWeights(1);
  	save_database_without_tree(database_dir);
}

// global variables used: cloud_names
void 
vtree_user::trace_directory(const char* dir, const char* prefix, std::vector<FeatureVector>& clouds, bool onlySaveClouds)
{
	ROS_INFO("Tracing directory: %s", dir);
	
	DIR *pdir = opendir(dir);
	struct dirent *pent = NULL;

	if (pdir == NULL) {
		ROS_ERROR("ERROR! Directory %s not found", dir);
		return;
	}
  
	while ((pent = readdir(pdir)))
	{
		if ((strcmp(pent->d_name, ".") != 0) && (strcmp(pent->d_name, "..") != 0) && (strcmp(pent->d_name, "IGNORE") != 0))
		{
			std::string short_filename(prefix);
			short_filename.append(pent->d_name);
			std::string filename(dir);
			filename.append(pent->d_name);
			struct stat st_buf;
			if (lstat(filename.c_str(), &st_buf) == -1)
			{
				ROS_ERROR("ERROR: Invalid file name %s", filename.c_str());
				ROS_ERROR("Exiting");
				exit(2);
			}

			if (S_ISDIR(st_buf.st_mode))
			{
				filename.append("/");
				short_filename.append("/");
				trace_directory(filename.c_str(), short_filename.c_str(), clouds, onlySaveClouds);
			}
			else
			{
				process_file(filename, clouds, onlySaveClouds);
				cloud_names.push_back(short_filename);
			}
		}
	}
	closedir(pdir);
}



// global variables used: NONE
void
vtree_user::process_file(std::string& filename, std::vector<FeatureVector>& clouds, bool onlySaveClouds )
{
	ROS_INFO("[vtree_user] Processing file %s...", filename.c_str());

	// Compute the features
	FeatureVector feature_vector;	
	compute_features( filename, feature_vector);
	
	if (!onlySaveClouds)
		clouds.push_back( feature_vector );
}


void 
vtree_user::compute_features( const std::string& cloud_filename,
								  		FeatureVector &feature_vector )
{
	typedef pcl16::PointXYZ nx_PointT;
	typedef pcl16::Normal nx_Normal;
	typedef pcl16::PointCloud<nx_PointT> nx_PointCloud;
	typedef pcl16::PointCloud<nx_Normal> nx_PointCloud_normal;
	typedef pcl16::PointCloud<int> nx_PointCloud_int;
	
	typedef pcl16::UniformSampling<nx_PointT> nx_Sampler;
	typedef pcl16::search::KdTree<nx_PointT> nx_SearchMethod;
	typedef pcl16::NormalEstimation<nx_PointT, nx_Normal> nx_NormalEst;
	
#if FEATURE == 1
	typedef pcl16::FPFHSignature33 nx_FeatureType;
	typedef pcl16::FPFHEstimation<nx_PointT, nx_Normal, nx_FeatureType> nx_FeatureEst;		
#elif FEATURE == 2
	typedef pcl16::PFHSignature125 nx_FeatureType;
	typedef pcl16::PFHEstimation<nx_PointT, nx_Normal, nx_FeatureType> nx_FeatureEst;		
#elif FEATURE == 3
	typedef pcl16::VFHSignature308 nx_FeatureType;
	typedef pcl16::VFHEstimation<nx_PointT, nx_Normal, nx_FeatureType> nx_FeatureEst;	
#else
	#error A valid feature definition is required!
#endif
	typedef pcl16::PointCloud<nx_FeatureType> nx_PointCloud_feature;

	// load the file
	nx_PointCloud::Ptr cld_ptr(new nx_PointCloud);
   pcl16::io::loadPCDFile<nx_PointT> ( cloud_filename, *cld_ptr);
   
   ROS_INFO("[vtree_user] Starting keypoint extraction...");
   clock_t tic = clock();
   nx_PointCloud::Ptr keypoints( new nx_PointCloud);
   nx_PointCloud_int::Ptr keypoint_idx(new nx_PointCloud_int);
   nx_Sampler uniform_sampling;
	uniform_sampling.setInputCloud ( cld_ptr );
	uniform_sampling.setRadiusSearch ( keypoint_radius_ );
	uniform_sampling.compute( *keypoint_idx );
	
	pcl16::copyPointCloud ( *cld_ptr, keypoint_idx->points, *keypoints);
	
	ROS_INFO("[vtree_user] No of Keypoints found %d", static_cast<int>(keypoint_idx->size()) );
	ROS_INFO("[vtree_user] Keypoint extraction took %f msec.", static_cast<double>((clock()-tic)*1000)/CLOCKS_PER_SEC );
	
	if( keypoints->empty() )
	{
		ROS_WARN("[vtree_user] No keypoints were found...");
		return;
	}
	
	// Compute normals for the input cloud
	ROS_INFO("[vtree_user] Starting normal extraction...");
	tic = clock();
	nx_PointCloud_normal::Ptr normals (new nx_PointCloud_normal);
	nx_SearchMethod::Ptr search_method_xyz (new nx_SearchMethod);
	nx_NormalEst norm_est;
	norm_est.setInputCloud ( cld_ptr );
	norm_est.setSearchMethod ( search_method_xyz );
	norm_est.setRadiusSearch ( normal_radius_ );
	norm_est.compute ( *normals );
	ROS_INFO("[vtree_user] Normal extraction took %f msec.", static_cast<double>((clock()-tic)*1000)/CLOCKS_PER_SEC );
	
	// Get features at the computed keypoints
	ROS_INFO("[vtree_user] Starting feature computation...");
	tic = clock();
	nx_PointCloud_feature::Ptr features(new nx_PointCloud_feature);
	nx_FeatureEst feat_est;
	feat_est.setInputCloud ( keypoints );
	feat_est.setSearchSurface ( cld_ptr );
	feat_est.setInputNormals ( normals );
	
	search_method_xyz.reset(new nx_SearchMethod);
	feat_est.setSearchMethod ( search_method_xyz );
	feat_est.setRadiusSearch ( feature_radius_ );
	feat_est.compute ( *features );
	ROS_INFO("[vtree_user] No of Features found %d", static_cast<int>(features->size()) );
	ROS_INFO("[vtree_user] Feature computation took %f msec.", static_cast<double>((clock()-tic)*1000)/CLOCKS_PER_SEC );
	
	// Rectify the historgram values to ensure they are in [0,100] and create a document
	for( nx_PointCloud_feature::iterator iter = features->begin();
			iter != features->end(); ++iter)
	{
		rectify_histogram( iter->histogram );			
		feature_vector.push_back( FeatureHist( iter->histogram ) );
	}
}


void
vtree_user::compute_features( const Eigen::Matrix<float,4,Eigen::Dynamic>& cloud_matrix_map,
								  		vt::Document &full_doc )
{
	typedef pcl16::PointXYZ nx_PointT;
	typedef pcl16::Normal nx_Normal;
	typedef pcl16::PointCloud<nx_PointT> nx_PointCloud;
	typedef pcl16::PointCloud<nx_Normal> nx_PointCloud_normal;
	typedef pcl16::PointCloud<int> nx_PointCloud_int;
	
	typedef pcl16::UniformSampling<nx_PointT> nx_Sampler;
	typedef pcl16::search::KdTree<nx_PointT> nx_SearchMethod;
	typedef pcl16::NormalEstimation<nx_PointT, nx_Normal> nx_NormalEst;
	
#if FEATURE == 1
	typedef pcl16::FPFHSignature33 nx_FeatureType;
	typedef pcl16::FPFHEstimation<nx_PointT, nx_Normal, nx_FeatureType> nx_FeatureEst;		
#elif FEATURE == 2
	typedef pcl16::PFHSignature125 nx_FeatureType;
	typedef pcl16::PFHEstimation<nx_PointT, nx_Normal, nx_FeatureType> nx_FeatureEst;		
#elif FEATURE == 3
	typedef pcl16::VFHSignature308 nx_FeatureType;
	typedef pcl16::VFHEstimation<nx_PointT, nx_Normal, nx_FeatureType> nx_FeatureEst;
#else
	#error A valid feature definition is required!
#endif
	typedef pcl16::PointCloud<nx_FeatureType> nx_PointCloud_feature;
	
	// copy the matrix into a pcl cloud
	nx_PointCloud::Ptr cld_ptr(new nx_PointCloud);
	for( int pt = 0; pt < cloud_matrix_map.cols(); ++pt)
	{
		cld_ptr->push_back( nx_PointT( cloud_matrix_map(0,pt), 
												cloud_matrix_map(1,pt),
												cloud_matrix_map(2,pt) ) );
	}
	
   ROS_INFO("[vtree_user] Starting keypoint extraction...");
   clock_t tic = clock();
   nx_PointCloud::Ptr keypoints( new nx_PointCloud);
   nx_PointCloud_int::Ptr keypoint_idx(new nx_PointCloud_int);
   nx_Sampler uniform_sampling;
	uniform_sampling.setInputCloud ( cld_ptr );
	uniform_sampling.setRadiusSearch ( keypoint_radius_ );
	uniform_sampling.compute( *keypoint_idx );
	
	pcl16::copyPointCloud ( *cld_ptr, keypoint_idx->points, *keypoints);
	
	ROS_INFO("[vtree_user] No of Keypoints found %d", static_cast<int>(keypoint_idx->size()) );
	ROS_INFO("[vtree_user] Keypoint extraction took %f msec.", static_cast<double>((clock()-tic)*1000)/CLOCKS_PER_SEC );
	
	if( keypoints->empty() )
	{
		ROS_WARN("[vtree_user] No keypoints were found...");
		return;
	}
	
	// Compute normals for the input cloud
	ROS_INFO("[vtree_user] Starting normal extraction...");
	tic = clock();
	nx_PointCloud_normal::Ptr normals (new nx_PointCloud_normal);
	nx_SearchMethod::Ptr search_method_xyz (new nx_SearchMethod);
	nx_NormalEst norm_est;
	norm_est.setInputCloud ( cld_ptr );
	norm_est.setSearchMethod ( search_method_xyz );
	norm_est.setRadiusSearch ( normal_radius_ );
	norm_est.compute ( *normals );
	ROS_INFO("[vtree_user] Normal extraction took %f msec.", static_cast<double>((clock()-tic)*1000)/CLOCKS_PER_SEC );
	
	// Get features at the computed keypoints
	ROS_INFO("[vtree_user] Starting feature computation...");
	tic = clock();
	nx_PointCloud_feature::Ptr features(new nx_PointCloud_feature);
	nx_FeatureEst feat_est;
	feat_est.setInputCloud ( keypoints );
	feat_est.setSearchSurface ( cld_ptr );
	feat_est.setInputNormals ( normals );
	
	search_method_xyz.reset(new nx_SearchMethod);
	feat_est.setSearchMethod ( search_method_xyz );
	feat_est.setRadiusSearch ( feature_radius_ );
	feat_est.compute ( *features );
	ROS_INFO("[vtree_user] No of Features found %d", static_cast<int>(features->size()) );
	ROS_INFO("[vtree_user] Feature computation took %f msec.", static_cast<double>((clock()-tic)*1000)/CLOCKS_PER_SEC );
	

	// Rectify the historgram values to ensure they are in [0,100] and create a document
	for( nx_PointCloud_feature::iterator iter = features->begin();
			iter != features->end(); ++iter)
	{
		rectify_histogram( iter->histogram );			
		full_doc.push_back( tree.quantize( FeatureHist( iter->histogram ) ) );
	}
}



void
vtree_user::rectify_histogram( float *histogram )
{
	int hist_size = sizeof( histogram ) / sizeof(float);
	//ROS_INFO("The feature histogram size is %d.", hist_size);
	
	for(int i=0; i<hist_size; ++i)
	{
		if ( ( std::isnan( histogram[i] ) ) || ( histogram[i] < 0.005) )
			histogram[i] = 0;
		else if ( histogram[i] > 100)
			histogram[i] = 100;	
	}
}

//****************************************************************************************
//************************* PROCESSING FUNCTIONS *****************************************
//****************************************************************************************

std::pair<float,std::string>
vtree_user::top_match( const Eigen::Matrix<float,4,Eigen::Dynamic>& cloud_matrix_map )
{
	std::vector<std::pair<float,std::string> > match_names;
	match_list( cloud_matrix_map, match_names, 1);
	return match_names.front();
	
	/*
	std::vector<std::pair<float,std::string> > match_names;
	std::vector<std::pair<float,std::string> > cluster_match_names;
	
	match_list( cloud_matrix_map, match_names, cluster_match_names, 1);
	
	if(enable_clustering)
		return cluster_match_names.front();
	else
		return match_names.front();
	*/
}

void 
vtree_user::match_list( const Eigen::Matrix<float,4,Eigen::Dynamic>& cloud_matrix_map,
						std::vector<std::pair<float,std::string> > & match_names,
						int num_match )
{
	// Extract keypoint in the pointcloud
	ROS_INFO("Extracting keypoints and computing features! We have a cloud with %d points", static_cast<int>(cloud_matrix_map.cols()) );
	vt::Document full_doc;
	compute_features( cloud_matrix_map, full_doc );
	
	// Obtain the matches from the database	
	vt::Matches matches;
	db->find(full_doc, num_match, matches);	// std::string documents_map[matches[i].id]->name; float matches[i].score, 
	
	match_names.clear();
	for ( vt::Matches::iterator it = matches.begin(); it != matches.end(); ++it)
		match_names.push_back( std::make_pair( it->score, documents_map[it->id]->name ));
}


/*
void 
vtree_user::match_list( const Eigen::Matrix<float,3,Eigen::Dynamic>& cloud_matrix_map,
						std::vector<std::pair<float,std::string> > & match_names,
						std::vector<std::pair<float,std::string> > & cluster_match_names,
						int num_match )
{

#if FEATURE == 1
	typedef pcl16::FPFHSignature33 nx_FeatureType;
	typedef pcl16::FPFHEstimation<nx_PointT, nx_Normal, nx_FeatureType> nx_FeatureEst;		
#elif FEATURE == 2
	typedef pcl16::PFHSignature125 nx_FeatureType;
	typedef pcl16::PFHEstimation<nx_PointT, nx_Normal, nx_FeatureType> nx_FeatureEst;		
#elif FEATURE == 3
	typedef pcl16::VFHSignature308 nx_FeatureType;
	typedef pcl16::VFHEstimation<nx_PointT, nx_Normal, nx_FeatureType> nx_FeatureEst;
#else
	#error A valid feature definition is required!
#endif

	// Extract keypoint in the pointcloud
	ROS_INFO("Extracting keypoints and computing features! We have a cloud with %d points", static_cast<int>(cloud_matrix_map.cols()) );
	vt::Document full_doc;
	compute_features( cloud_matrix_map, full_doc );
	

	// Cluster the keypoints in 2D
	ANNpointArray ann_points;
	ann_points = annAllocPts(num_feat, 3);
	std::vector<KeypointExt*> extended_keypoints;
		
	for( int i=0; i < num_feat; ++i )
	{
		if (enable_clustering)
	  	{
	  		ann_points[i][0] = keypoint_cloud->points[i].x;
	  		ann_points[i][1] = keypoint_cloud->points[i].y;
	  		ann_points[i][2] = keypoint_cloud->points[i].z;
	  	}
		extended_keypoints.push_back( new KeypointExt( feature_cloud->at(i), full_doc[i] ) );
	}
	
	
	int cluster_count = 0;
	std::vector<int> cluster_sizes;
	if (enable_clustering)
	{
		std::vector<int> membership(num_feat);
		cluster_count = pcd_utils::cluster_points( ann_points, num_feat, membership,
												  radius_adaptation_r_max, radius_adaptation_r_min, 
												  radius_adaptation_A, radius_adaptation_K );
																		
		cluster_sizes.resize(cluster_count, 0);
		//cluster_sizes.assign(cluster_count, 0);
		for (int i = 0; i < num_feat; ++i)
		{
			extended_keypoints[i]->cluster = membership[i];
			++cluster_sizes[membership[i]];
		}
		delete[] ann_points;
	}
	if(DEBUG)
		ROS_INFO_STREAM("Clusters found = " << cluster_count);


	// Obtain the matches from the database	
	vt::Matches matches;
	db->find(full_doc, num_match, matches);	// std::string documents_map[matches[i].id]->name; float matches[i].score, 
	
	match_names.clear();
	for ( vt::Matches::iterator it = matches.begin(); it != matches.end(); ++it)
		match_names.push_back( std::make_pair( it->score, documents_map[it->id]->name ));
		

	if (enable_clustering)
	{
		// store in matches_map
		std::map<uint32_t, float> matches_map;
		for ( vt::Matches::iterator it = matches.begin(); it != matches.end(); ++it)
		{
			matches_map[it->id] = it->score;
		}
	
		// Calculates and accumulates scores for each cluster
		for (int c = 0; c < cluster_count; ++c) 
		{
			vt::Document cluster_doc;
			vt::Matches cluster_matches;
		
			for (int i = 0; i < num_feat; ++i)
				if ( extended_keypoints[i]->cluster == static_cast<unsigned int>(c) )
					cluster_doc.push_back(full_doc[i]);
		
			if (cluster_doc.size() < static_cast<unsigned int>(min_cluster_size))
				continue;
		
			db->find(cluster_doc, num_match, cluster_matches);
		
			if(DEBUG)
				ROS_INFO_STREAM("Cluster " << c <<  "(size = " << cluster_doc.size() << "):");

		
			//update_matches_map(cluster_matches, cluster_doc.size());
			for ( vt::Matches::iterator it = cluster_matches.begin(); it != cluster_matches.end(); ++it)
			{
				matches_map[it->id] = it->score;
			}		
		}
		
		// Get the updated match names
		cluster_match_names.clear();
		for( std::map<uint32_t, float>::iterator iter = matches_map.begin();
		 	 iter != matches_map.end(); ++iter )
		{
			cluster_match_names.push_back( std::make_pair( iter->second, documents_map[iter->first]->name) );
		}
		
		// sort
		std::sort( cluster_match_names.begin(), cluster_match_names.end() );
	}
	
}
*/

ros_vtree_user::ros_vtree_user( ros::NodeHandle & nh )
	: vtree_user("/load","", "", 16, 5), nh_(nh), private_nh_("~")
{
	// Tree parameters
	private_nh_.param ("DEBUG", DEBUG, false);
	private_nh_.param ("command", command, std::string("/load"));
	private_nh_.param ("database_dir", database_dir, std::string("/media/natanaso/Data/Stuff/Research/git/icra_2013/ns_shared/penn_nbv_slam/vision_module/data"));
	private_nh_.param ("clouds_dir", clouds_dir, std::string("/media/natanaso/Data/Stuff/Research/git/icra_2013/ns_shared/penn_nbv_slam/data/cloud_data"));
	private_nh_.param ("tree_k", tree_k, 16);
	private_nh_.param ("tree_levels", tree_levels, 5);

	// Keypoint and Feature parameters
	private_nh_.getParam("keypoint_radius", keypoint_radius_);
	private_nh_.getParam("normal_radius", normal_radius_);
	private_nh_.getParam("feature_radius", feature_radius_);

	// Clustering parameters
	private_nh_.param ("enable_clustering", enable_clustering, 0);
	private_nh_.param ("min_cluster_size", min_cluster_size, 30);
	private_nh_.param ("radius_adaptation_r_min", radius_adaptation_r_min, 200.0);
	private_nh_.param ("radius_adaptation_r_max", radius_adaptation_r_max, 600.9);
	private_nh_.param ("radius_adaptation_K", radius_adaptation_K, 0.02);
	private_nh_.param ("radius_adaptation_A", radius_adaptation_A, 800.0);
}
	
	

	


