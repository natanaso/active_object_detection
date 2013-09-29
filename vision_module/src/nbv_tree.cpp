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
 * @b comprises of all definitions for nbv tree class
 */
#include "vision_module/nbv_tree.h"
#include "ros/package.h"
#include "vision_module/util.h"
#include <pcl/keypoints/uniform_sampling.h>

#include <time.h>

const int VIEWS_TO_SHOW = 1;
////////////////////////////
DocumentInfo::DocumentInfo() :
	delete_document(false) 
{
}

//////////////////////////////////////////////////////////////////////
DocumentInfo::DocumentInfo(vt::Document* document, std::string& name) :
	 delete_document(false), document(document), name(name) 
{
}

////////////////////////////
DocumentInfo::~DocumentInfo() 
{
	if (delete_document)
		delete[] document;
}

///////////////////////////////////////////
void DocumentInfo::write(std::ostream& out) 
{
	size_t length = name.length();
	out.write((char*) &length, sizeof(size_t));
	out.write(name.c_str(), name.length());
	size_t doc_length = document->size();
	out.write((char*) &doc_length, sizeof(size_t));
	out.write((char*) &(document->at(0)), doc_length * sizeof(vt::Word));
}

/////////////////////////////////////////
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

/////////////////////////////////////////////////////////////////////////////////////
NBVTree::NBVTree() :
	tree_builder(Feature::Zero())
  {
	command = std::string("/load");
	database_location = std::string("database/next_best_view");
	clouds_directory = std::string("data/next_best_view");
	votes_count = 30;
	tree_k = 5;
	tree_levels = 5;
	min_cluster_size = 30;
	unknown_object_threshold = 0.3;
	sliding_window_size = 10;
	enable_clustering = 1;
	enable_incremental_learning = 0;
	object_id = 700000;
	frame_number = 0;
	radius_adaptation_r_min = 200.0;
	radius_adaptation_r_max = 600.9;
	radius_adaptation_A = 800.0;
	radius_adaptation_K = 0.02;
}


//////////////////////
NBVTree::~NBVTree()
{
	delete db;
	std::map<int, DocumentInfo*>::iterator iter;
	for (iter = documents_map.begin(); iter != documents_map.end(); ++iter)
		delete[] iter->second;
}

///////////////////////////////////////////////////////////////
std::string NBVTree::process_pointcloud(PointCloud::Ptr point_cloud_in)
{
	//extract keypoints in the whole image
	ROS_INFO("call keypoints extractor!");
	std::vector<Keypoint_p> keypoints = extract_keypoints(point_cloud_in);
	std::vector<Keypoint_p> p = keypoints;
	pc_keypoints_count = 0;

	vt::Document full_doc;
	//push keypoints in the vocabulary tree document
	 for(size_t i=0; i < keypoints.size();i++)
		  {
		  for(int j =0; j<125;j++)
			  {
			  if(keypoints[i].histogram[j] < 0.005){
				  keypoints[i].histogram[j] = 0;}
			  else
				  if(keypoints[i].histogram[j] > 100){
					  keypoints[i].histogram[j] = 100;}
				  else{
					  if(std::isnan(keypoints[i].histogram[j])){
						  keypoints[i].histogram[j] = 0;}
			          }
			  }
			//std::cerr<<"Histogram "<<keypoints[i]<<std::endl;
			Feature f(keypoints[i].histogram);
			full_doc.push_back(tree.quantize(f));
			//std::cerr<<"Displaying feature: "<<f<<std::endl;
			++pc_keypoints_count;
		  }

	ROS_INFO("%d keypoints found!",pc_keypoints_count);

	ANNpointArray points;
	points = annAllocPts(pc_keypoints_count, 3);

	//initialize a vector of objects of KeypointExt (keypoints extension class)
	std::vector<KeypointExt*> pc_keypoints(pc_keypoints_count);

	 for(size_t i=0; i < keypoints.size();i++)
		  {
		  if (enable_clustering)
			  {
			  points[i][0] = keypoints_->points[i].x;
			  points[i][1] = keypoints_->points[i].y;
			  points[i][2] = keypoints_->points[i].z;
			  }
		  pc_keypoints[i] = new KeypointExt(keypoints[i], full_doc[i]);
		  }
  
	size_t cluster_count = 0;


	// if clustering enabled - group features in 2D
    // according to adaptive radius criterion
	// Think about how this clustering needs to happen
	// perform hierarchical kmeans before insertion into database??
	if (enable_clustering)
		{
		std::vector<int> membership(pc_keypoints_count);
		cluster_count = cluster_points(points, pc_keypoints_count,
                                   membership, radius_adaptation_r_max,
                                   radius_adaptation_r_min, radius_adaptation_A,
                                   radius_adaptation_K);
		if(DEBUG)
		ROS_INFO_STREAM("Clusters found = " << cluster_count);
		cluster_sizes.resize(cluster_count, 0);
		cluster_sizes.assign(cluster_count, 0);
		for (size_t i = 0; i < pc_keypoints_count; ++i)
		  {
			pc_keypoints[i]->cluster = membership[i];
			++cluster_sizes[membership[i]];
		  }
		delete[] points;
	  }

	matches_map.clear();

	// Search the whole pointcloud
	// vector of Matches

	vt::Matches matches;

	//find #votes_count matches
	db->find(full_doc, votes_count + 1, matches);
	if(DEBUG)
	ROS_INFO("Whole Point Cloud: ");


	//calculate scores over the entire input pointcloud
	// @TODO - Why is calculation over the pointcloud necessary?
	update_matches_map(matches, pc_keypoints_count);

	// Calculates and accumulates scores for each cluster
	for (size_t c = 0; c < cluster_count; ++c) 
	  {
		vt::Document cluster_doc;
		vt::Matches cluster_matches;
    
		for (size_t i = 0; i < pc_keypoints_count; ++i)
			if (pc_keypoints[i]->cluster == c)
				cluster_doc.push_back(full_doc[i]);
    
		if (cluster_doc.size() < (size_t)min_cluster_size)
			continue;
    
		db->find(cluster_doc, votes_count + 1, cluster_matches);
		if(DEBUG)
		ROS_INFO_STREAM("Cluster " << c <<  "(size = " << cluster_doc.size() << "):");
		update_matches_map(cluster_matches, cluster_doc.size());
	  }

	// Sort the votes
	std::vector<std::pair<uint32_t, float> > votes(matches_map.size());
	std::map<uint32_t, float>::iterator iter = matches_map.begin();
	for (int i = 0; iter != matches_map.end(); ++iter, ++i) 
	  {
		votes[i].first = iter->first;
		votes[i].second = iter->second;
	  }

	//print results
	if(DEBUG){
	ROS_INFO("RESULTS: ");
	ROS_INFO("votes_count: %d Documents Size %d: ",votes_count,(int)documents_map.size());}
	std::sort(votes.begin(), votes.end(), compare_pairs);

	// if object is unknown either skip detection
	// or add the template to the database
	double max_score = 0.0;
	best_score = 0.0;
	if (votes[0].second < unknown_object_threshold) 
	  {
		ROS_INFO("Unknown object!");
		if (enable_incremental_learning) 
		  {
			time_t rawtime;
			struct tm * timeinfo;
			char buffer[25];
			time(&rawtime);
			timeinfo = localtime(&rawtime);
    		strftime(buffer, 25, "Object_%Y%m%d%H%M%S", timeinfo);
			std::string object_name(buffer);
			add_pointcloud_to_database(full_doc, object_name);
			ROS_INFO("Object added into database as %s!", object_name.c_str());
		  }
	  } 
	else 
	  {
		for (int i = 0; (i < votes_count && i < (int)documents_map.size()); ++i){
			//if(DEBUG)
				ROS_INFO("%s, %f", documents_map[votes[i].first]->name.c_str(),votes[i].second);

			if(votes[i].second > max_score)
				max_score = votes[i].second;
			//if best score queried by user to build observation model
			if(get_match)
			{
				size_t found = documents_map[votes[i].first]->name.find(match_name);
				if(found!=std::string::npos){
					if(DEBUG)
					ROS_INFO("We have an exact match!!");
					if(votes[i].second > best_score)
						{
						  best_name.clear(); // resetting the string
						  best_score =votes[i].second;
						  best_name = documents_map[votes[i].first]->name;


						  // pruning to best name match
						  size_t last_slash = best_name.find_last_of("/");
						  if (last_slash == std::string::npos)
							  last_slash = 0;
						  else
							  ++last_slash;

						  //return the name of the object, that is the correponding pointcloud name
						  if(DEBUG)
						  ROS_INFO("ID of best match score : %s",best_name.c_str());
						  best_name = best_name.substr(last_slash, best_name.find_last_of('.') - last_slash);
						}
					}
			}

		}

		//normalizing the match score
		if(max_score > 0)
			best_score /= max_score;

		if(best_score < 0.01)
			best_score  = 0.01;

    
		// classify with the sliding window
		std::string name;

    	std::set<std::string> unique_names;
		int added = 0;
		for (int i=0; added<VIEWS_TO_SHOW && i < votes_count; ++i)
			{
			DocumentInfo* d = documents_map[votes[i].first];
			size_t position = d->name.find('_');

			if (position == std::string::npos)
				position = d->name.find('.');

			if(DEBUG)
			ROS_INFO("Name of the best match document : %s",d->name.c_str());
			name = d->name;
			}

		//also for logging
		size_t last_slash_id = name.find_last_of("/");
		if (last_slash_id == std::string::npos)
			last_slash_id = 0;
		else
			++last_slash_id;

		//return the name of the object, that is the correponding pointcloud name
		ROS_INFO("Return ID of best match document : %s",name.c_str());

		return name.substr(last_slash_id, name.find_last_of('.')
                       - last_slash_id).c_str();
	  }

	keypoints.clear();

	return "";
}

//////////////////////
int NBVTree::start()
{
	//if init build and save the database
	if (command.compare("/init") == 0)
		{
		build_database(clouds_directory);
		save_database(database_location);
		}
	//load previously built database and perform recognition
	else if (command.compare("/load") == 0)
		load_database(database_location);

	//only extract features and save them in a specified images_directory
	else if (command.compare("/features_only") == 0)
		process_pointclouds(clouds_directory);
	else
		return 1;

	return 0;
}

/////////////////////////////////////////////////////
void NBVTree::build_database(std::string directory)
{

	std::vector<FeatureVector> clouds;
	trace_directory(directory.c_str(), "", clouds);
	ROS_INFO("Preparing features for the tree...");

	FeatureVector all_features;

	for (int i = 0; i < clouds.size(); ++i)
		for (int j = 0; j < clouds[i].size(); ++j)
			{
			all_features.push_back(clouds[i][j]);
			}

  
	ROS_INFO_STREAM("Building a tree with " << all_features.size() << " nodes...");
	tree_builder.build(all_features, tree_k, tree_levels);
	tree = tree_builder.tree();
	ROS_INFO("Creating the documents...");
	docs.resize(clouds.size());

	for (int i = 0; i < clouds.size(); ++i)
		for (int j = 0; j < clouds[i].size(); ++j)
			docs[i].push_back(tree.quantize(clouds[i][j]));
  
	ROS_INFO("Creating database...");
	db = new vt::Database(tree.words());

	ROS_INFO("Populating the database with the documents...");
	for (int i = 0; i < clouds.size(); ++i)
		documents_map[db->insert(docs[i])] = new DocumentInfo(&(docs[i]),cloud_names[i]);

	ROS_INFO("Training database...");
	db->computeTfIdfWeights(1);
	ROS_INFO("Database created!");
}

/////////////////////////////////////////////////////
void NBVTree::process_pointclouds(std::string directory)
{
	std::vector<FeatureVector> clouds;
	trace_directory(directory.c_str(), "", clouds, true);
}

/////////////////////////////////////////////////////////////////
void NBVTree::save_database_without_tree(std::string& directory)
{
	ROS_INFO("Saving documents...");
	std::string documents_file(directory);
	documents_file.append("/clouds.documents");

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
	std::string weights_file(directory);

	weights_file.append("/clouds.weights");
	db->saveWeights(weights_file.c_str());
	out.close();
	ROS_INFO("Done! Press Ctrl+C and roslaunch detect.launch");
}

/////////////////////////////////////////////////////
void NBVTree::save_database(std::string& directory)
{
	ROS_INFO("Saving the tree...");
	std::string tree_file(directory);
	tree_file.append("/clouds.tree");
	tree.save(tree_file.c_str());
	save_database_without_tree(directory);
}


/////////////////////////////////////////////////////
void NBVTree::load_database(std::string& directory)
{
	ROS_INFO("Loading the tree...");
	std::string tree_file(directory);
	tree_file.append("/clouds.tree");
	tree.load(tree_file.c_str());

	ROS_INFO("Initializing the database...");
	db = new vt::Database(tree.words());
	std::string documents_file(directory);
	documents_file.append("/clouds.documents");
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
	std::string weights_file(directory);
	weights_file.append("/clouds.weights");
	db->loadWeights(weights_file.c_str());
	in.close();
	ROS_INFO("READY!");
}

///////////////////////////////////////////////////////////////////////////
void NBVTree::add_pointcloud_to_database(vt::Document& doc, std::string& name)
{
	docs.push_back(doc);
	documents_map[db->insert(doc)] = new DocumentInfo(&doc, name);
	db->computeTfIdfWeights(1);
  	save_database_without_tree(database_location);
}

///////////////////////////////////////////////////////////////////////////////////////
void NBVTree::trace_directory(const char* dir, const char* prefix,
                                std::vector<FeatureVector>& clouds, bool onlySaveClouds)
{
	if(DEBUG)
		ROS_INFO("Tracing directory: %s", dir);
	DIR *pdir = opendir(dir);
	struct dirent *pent = NULL;

	if (pdir == NULL) {
		ROS_ERROR("ERROR! Directory %s not found", dir);
		return;
	}
  
	while ((pent = readdir(pdir))) {
		if (strcmp(pent->d_name, ".") != 0 && strcmp(pent->d_name, "..") != 0
				&& strcmp(pent->d_name, "IGNORE") != 0)
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
				trace_directory(filename.c_str(), short_filename.c_str(),clouds, onlySaveClouds);
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

/////////////////////////////////////////////////////////////////////
void NBVTree::update_matches_map(vt::Matches& matches, size_t size)
{
	if(DEBUG)
		ROS_INFO("Number of registered matches :%d",(int)matches.size());
	for (int i = 0; (i < votes_count && i < (int)matches.size()); ++i) 
		{
		if (matches_map.count(matches[i].id) == 0)
			matches_map[matches[i].id] = 0;
		int next_i = i;
		float diff;

		do
			{
			++next_i;
			diff = matches[next_i].score - matches[i].score;
			}
		while (diff == 0 && next_i < votes_count);

		float database_score = 2 - matches[i].score;
		float place_score = 1;
		float size_score = (float)size;
		float score = database_score * place_score * size_score;
		float test_score = database_score * matches[i].id * size_score;

		matches_map[matches[i].id] = ((int)matches.size()-1) - i;
		if(DEBUG)
			ROS_INFO("\t%f\t%f\t%f\t%d\t%s", matches[i].score, diff, score, (int)matches.size() - i ,
             documents_map[matches[i].id]->name.c_str());
		}
}

/////////////////////////////////////////////////////////////////////////////////////
void NBVTree::process_file(std::string& filename,
                             std::vector<FeatureVector>& clouds, bool onlySaveClouds)
{
	if(DEBUG)
		ROS_INFO("Processing file %s...", filename.c_str());

	PointCloud::Ptr cloud(new PointCloud);

    pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud);

    std::vector<Keypoint_p> keypoints = extract_keypoints(cloud);

    if(DEBUG)
		ROS_INFO("Keypoints extracted,");
	FeatureVector features;
	int count = 0;

	  for(size_t i=0; i < keypoints.size();i++)
		  {
		  for(int j =0; j<125;j++)
			  {
			  if(keypoints[i].histogram[j] < 0.005){
				  keypoints[i].histogram[j] = 0;}
			  else
				  if(keypoints[i].histogram[j] > 100){		// when can we have a bin value of more than 100?
					  keypoints[i].histogram[j] = 100;}
				  else{
					  if(std::isnan(keypoints[i].histogram[j])){
						  keypoints[i].histogram[j] = 0;}
			          }
			  }
 			Feature f(keypoints[i].histogram);
			features.push_back(f);
			count++;
		  }


	if (!onlySaveClouds)
		clouds.push_back(features);
	keypoints.clear();

	if(DEBUG)
		ROS_INFO("Done! %d features found!", count);
}

///////////////////////////////////////////////////////////////////////
std::vector<Keypoint_p> NBVTree::extract_keypoints(PointCloud::Ptr pointcloud, bool frames_only)
{
	LocalFeatures_p::Ptr phfs(new LocalFeatures_p());

	// First computing the normals
	SurfaceNormals::Ptr normals_(new SurfaceNormals);
	SearchMethod::Ptr search_method_xyz_ (new SearchMethod);
    float normal_radius_ (0.03f);
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    float feature_radius_ (0.04f);	// NOTE: PFH rad must be larger than the normal radius
    								// FPFH is faster

	//ROS_INFO("Starting normal extraction...");
	//clock_t tic = clock();
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setInputCloud (pointcloud);
	norm_est.setSearchMethod (search_method_xyz_);
	norm_est.setRadiusSearch (normal_radius_);
	norm_est.compute (*normals_);
	//ROS_INFO("Normal extraction took %f msec.", static_cast<double>((clock()-tic)*1000)/CLOCKS_PER_SEC );
	
	//ROS_INFO("Starting keypoint extraction...");
	//tic = clock();
	keypoints_.reset(new PointCloud);

	compute_keypoints (pointcloud, keypoints_);

	//ROS_INFO("Keypoint extraction took %f msec.", static_cast<double>((clock()-tic)*1000)/CLOCKS_PER_SEC );
	
	
	//ROS_INFO("Starting feature computation...");
	//tic = clock();
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_est;

	pfh_est.setSearchMethod (search_method_xyz_);
	pfh_est.setRadiusSearch (feature_radius_);

	pfh_est.setSearchSurface (pointcloud);
	pfh_est.setInputNormals (normals_);
	
	std::vector<Keypoint_p> kpt;

	if((int)keypoints_->points.size() > 0){

		pfh_est.setInputCloud (keypoints_);

		pfh_est.compute (*phfs);

	    for(int pit = 0; pit<=phfs->points.size();pit++)
	    	{
	    	kpt.push_back(phfs->points[pit]);
	    	}
	}
	//ROS_INFO("Feature computation took %f msec.", static_cast<double>((clock()-tic)*1000)/CLOCKS_PER_SEC );
	
	return(kpt);

}
void NBVTree::compute_keypoints (pcl::PointCloud<PointT>::Ptr &points, pcl::PointCloud<PointT>::Ptr &keypoints)
{	
	float model_ss_ (0.025f);

	pcl::PointCloud<int> keypoint_indices;

	pcl::UniformSampling<PointT> uniform_sampling;
	uniform_sampling.setInputCloud (points);
	uniform_sampling.setRadiusSearch (model_ss_);
	uniform_sampling.compute (keypoint_indices);
	//ROS_INFO("No of Keypoints found %d",(int)keypoint_indices.points.size ());
	pcl::copyPointCloud (*points, keypoint_indices.points, *keypoints);
}

/*
// Compute the resolution of a cloud
double
NBVTree::computeCloudResolution (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud (cloud);

	for (size_t i = 0; i < cloud->size (); ++i)
	{
		if (! pcl_isfinite ((*cloud)[i].x))
		{
		  continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
		if (nres == 2)
		{
		  res += sqrt (sqr_distances[1]);
		  ++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}
*/

