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
#include <std_msgs/String.h>
#include "vision_module/nbv_tree.h"
#include "ros/package.h"
#include <stdlib.h>
#include <fstream>
#include <iomanip>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ctype.h>
#include <vector>
#include <algorithm>

const int VIEWS_TO_SHOW = 1;

NBVTree::NBVTree(ros::NodeHandle &anode) :
	nh_(anode), tree_builder(Feature::Zero())
  {
    //setting base dir
    //nh_.param ("base_dir", base_dir_, std::string("/home/bharath/GRASP_Code"));
    nh_.param ("base_dir", base_dir_, std::string("/media/natanaso/Data/Stuff/Research/git/icra_2013"));
    //load or build database, equivalent to detect or build
    nh_.param ("command", command, std::string("/load"));
    //path to database
    nh_.param ("database_location", database_location, std::string(base_dir_+"/ns_shared/penn_nbv_slam/vision_module/data"));
    //path to training images
    nh_.param ("clouds_folder", clouds_directory, std::string(base_dir_+"/ns_shared/penn_nbv_slam/data/cloud_data"));
    //path to viewpoints file
    nh_.param ("viewpoint_file", viewpoint_file, std::string(base_dir_+"/ns_shared/penn_nbv_slam/vision_module/data/viewpoints.txt"));
    //path to viewpoint correspondence file
    nh_.param ("correspondence_file", correspondence_file, std::string(base_dir_+"/ns_shared/penn_nbv_slam/vision_module/data/correspondence.txt"));
    //path to visualization images
    //how many best votes do we consider
    nh_.param ("votes_count", votes_count, 100);
    //vocabulary trees parameters
    nh_.param ("tree_k", tree_k, 16);
    nh_.param ("tree_levels", tree_levels, 5);
    //min size of features when clustering
    nh_.param ("min_cluster_size", min_cluster_size, 30);
    //when is object uknown (score goes from 0 (best) - 2 (worst)
    nh_.param ("unknown_object_threshold", unknown_object_threshold, 0.3);
    //wait for #sliding_window_size frames before deciding on the classification result
    nh_.param ("sliding_window_size", sliding_window_size, 10);
    //clustering yes or no
    nh_.param ("enable_clustering", enable_clustering, 1);
    nh_.param ("enable_incremental_learning", enable_incremental_learning, 0);
    //enable visualization
    nh_.param ("get_match", get_match, true);
    nh_.param ("obj_present", obj_present, true);
    nh_.param ("match_name", match_name, std::string("handlebottle"));
    nh_.param ("multi_hypothesis", multi_hypothesis, true);
    //radius adaptation parameters for clustering
    nh_.param ("radius_adaptation_r_min", radius_adaptation_r_min, 200.0);
    nh_.param ("radius_adaptation_r_max", radius_adaptation_r_max, 600.9);
    nh_.param ("radius_adaptation_A", radius_adaptation_A, 800.0);
    nh_.param ("radius_adaptation_K", radius_adaptation_K, 0.02);
    nh_.param ("extract_roi" , extract_roi_, false);
    nh_.param ("detect_objects", detect_objects_, true);
    nh_.param ("DEBUG", DEBUG, false);
    nh_.param ("hypotheses_no" , hypotheses_no_, 7);
    nh_.param ("views_no", views_no_, 48);

    //initializing best score
    best_score = 0.0;
}

void NBVTree::load_matrix(MatrixXf & values ,std::string filename,int row, int col)
	{
		  std::ifstream in(filename.c_str());

		  if (!in) {
		    std::cout << "Cannot open file.\n";
		  }

		  double distances[row][col];

		  for (int x = 0; x < row; x++) {
		    for (int y = 0; y < col; y++) {
		      in >> distances[x][y];
		    }
		  }

		  for (int x = 0; x < row; x++) {
		    for (int y = 0; y < col; y++) {
		      values(x,y) =  distances[x][y];
		    }
		  }

		  in.close();
	}

void NBVTree::load_matrix(boost::multi_array<double,3> & values ,std::string filename,int row, int col, int third)
	{
		  std::ifstream in(filename.c_str());

		  ROS_INFO("File name is %s",filename.c_str());

		  if (!in) {
		    std::cout << "Cannot open file.\n";
		  }

		  double distances[row][col][third];

		  for (int x = 0; x < row; x++) {
		     for(int z = 0; z < third;z++){
				    for (int y = 0; y < col; y++) {
				      in >> distances[x][y][z];
				    }

		     }
		  }


		  for (int x = 0; x < row; x++) {
		    for (int y = 0; y < col; y++) {
		    	for(int z = 0; z < third; z++){
		    		values[x][y][z] =  distances[x][y][z];
		    	}
		    }

		  }

		  in.close();
	}

int
NBVTree::get_vp_score( PointCloud::Ptr point_cloud_in )
{
	std::string top_match = top_match_name( point_cloud_in );
	
	// Display
	ROS_INFO( "The top match is: %s", top_match.c_str() );
	
	// Here we can check for several match_names
	return get_vp_score( top_match, match_name );
}

int
NBVTree::get_vp_score( std::string top_match, std::string query )
{
	int vp = -1;
	
	size_t found = top_match.find(query);
	if(found!=std::string::npos){
		std::size_t underscore_pos = top_match.find_last_of("_");
		std::size_t dot_pos = top_match.find_last_of(".");
		vp = atoi(top_match.substr(underscore_pos+1, dot_pos-1).c_str());
	}
	return vp;
}

std::string 
NBVTree::top_match_name( PointCloud::Ptr point_cloud_in )
{
	std::vector<std::pair<float, std::string> > match_names;
	match_list( point_cloud_in, match_names);
	
	return match_names[0].second;
}

void 
NBVTree::match_list( PointCloud::Ptr point_cloud_in, std::vector<std::pair<float, std::string> > & match_names )
{
	
	// Extract keypoint in the pointcloud
	ROS_INFO("Call keypoints extractor! We have a cloud with %d points", point_cloud_in->size() );
	std::vector<Keypoint_p> keypoints = extract_keypoints( point_cloud_in );
	//ROS_INFO("Keypoints extracted!");
	
	if(keypoints.size() == 0)
		return;
	//std::vector<Keypoint_p> p = keypoints;
	
	// Create a document from the keypoints
	vt::Document full_doc;	
	pc_keypoints_count = 0;	
	for(size_t i=0; i < keypoints.size();i++)
	{
		for(int j =0; j<125;j++)
		{
			if(keypoints[i].histogram[j] < 0.005)
			{
				keypoints[i].histogram[j] = 0;
		  	}else if(keypoints[i].histogram[j] > 100)
		  	{
				keypoints[i].histogram[j] = 100;
			}else if(std::isnan(keypoints[i].histogram[j]))
			{
				keypoints[i].histogram[j] = 0;
		    }
		}
		//std::cerr<<"Histogram "<<keypoints[i]<<std::endl;
		Feature f(keypoints[i].histogram);
		full_doc.push_back(tree.quantize(f));
		//std::cerr<<"Displaying feature: "<<f<<std::endl;
		++pc_keypoints_count;
	}
	ROS_INFO("%d keypoints found!",pc_keypoints_count);
	

	
	// Cluster the keypoints in 2D
	ANNpointArray points;
	points = annAllocPts(pc_keypoints_count, 3);
	
	std::vector<KeypointExt*> pc_keypoints(pc_keypoints_count);
	
	
	// keypoints_ are the locations of the computed features!
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

	
	
	// Obtain the matches from the database
	matches_map.clear();
	vt::Matches matches;
	db->find(full_doc, votes_count + 1, matches);
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
	  
	// Get the match names
	for( std::map<uint32_t, float>::iterator iter = matches_map.begin();
		 iter != matches_map.end(); ++iter )
	{
		match_names.push_back( std::make_pair( iter->second, documents_map[iter->first]->name) );
	}
	
	// sort the match names according to the matches in descending order
	std::sort( match_names.rbegin(), match_names.rend() );
}

bool
NBVTree::process_pointcloud_simple( PointCloud::Ptr point_cloud_in, int &hypothesis,
																    int &viewpoint)
{
	//extract keypoints in the whole image
	ROS_INFO("Call keypoints extractor! We have a cloud with %d points", point_cloud_in->size() );
	std::vector<Keypoint_p> keypoints = extract_keypoints( point_cloud_in );

	if(keypoints.size() > 0)
	{

	std::vector<Keypoint_p> p = keypoints;
	pc_keypoints_count = 0;
	ROS_INFO("Keypoints extracted!");
	
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

		// check if it is a hypothesis zero
		size_t found = documents_map[votes[0].first]->name.find(match_name);

		if(found!=std::string::npos){

			// finding view
			int hyp[]={0,60,120,180,240,300};
			for(int i = 0; i<6 ; i++){
				std::stringstream hyp_search;
				hyp_search<<match_name<<"_yaw"<<hyp[i];

				size_t found_hyp = documents_map[votes[0].first]->name.find(hyp_search.str());
				if(found_hyp!= std::string::npos){
					for(int j = 0; j<30 ;j++){

						std::stringstream view_search;
						view_search<<match_name<<"_yaw"<<hyp[i]<<"_vp_"<<j<<".pcd";
						size_t found_view = documents_map[votes[0].first]->name.find(view_search.str());
						if(found_view!= std::string::npos){
							hypothesis = i + 1;
							viewpoint = j;
						}
					}

				}
			}
		}
		else{
			hypothesis = 0;
			viewpoint = 0;

		}

	  }

	keypoints.clear();
	return true;
	}
	else
		return false;

}

/*
std::string NBVTree::process_pointcloud(PointCloud::Ptr point_cloud_in,std::string filename)
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

			Feature f(keypoints[i].histogram);
			full_doc.push_back(tree.quantize(f));
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
	ROS_INFO("RESULTS: ");
	std::sort(votes.begin(), votes.end(), compare_pairs);

	// if object is unknown either skip detection
	// or add the template to the database
	double max_score = 0.0;
	std::string max_name;
	size_t found;

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

		// query entryname to figure out view point
		int query_view = 0;
		for (int i = 0; i < views_no_;i++){
			std::stringstream query_name;
			query_name<<"_"<<i<<".pcd";
			size_t found_wrd;

			// different member versions of find in the same order as above:
			found_wrd=filename.find(query_name.str());

			if (found_wrd!=std::string::npos){
				query_view = i;
			}

		}

		// find the corresponding viewpoint
		// use the score map
		MatrixXf view_correspondence(30,6);
		load_matrix(view_correspondence,correspondence_file,30,6);

		// now getting the corresponding view
		int view = view_correspondence(query_view,yaw_) - 1;

		std::stringstream search_name;
		search_name<<match_name<<"_"<<query_view<<".pcd";

		for (int i = 0; (i < votes_count && i < (int)documents_map.size()); ++i){
			if(DEBUG)
				ROS_INFO("%s, %f", documents_map[votes[i].first]->name.c_str(),votes[i].second);

			if(votes[i].second > max_score)
			{
				max_name.clear();
				max_score = votes[i].second;
				max_name = documents_map[votes[i].first]->name;
			}

			//if best score queried by user to build observation model

			// Need to do something more smarter for the match name
			if(get_match)
			{
				//loop over all query patterns

				if(multi_hypothesis){
					found = documents_map[votes[i].first]->name.find(search_name.str());
				}
				else{
					found = documents_map[votes[i].first]->name.find(match_name);
				}



				if(found!= std::string::npos){
					if(DEBUG){
					ROS_INFO("We have an exact match!!");
					ROS_INFO("Query name : %s", search_name.str().c_str());}
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
		if(enable_confusion){
			best_name.clear();
			best_name = max_name;
			best_score = max_score/max_score;
		}
		else{
			if(max_score > 0)
				best_score /= max_score;

			if(best_score < 0.01)
				best_score  = 0.01;
		}



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

			name = d->name;
			}

		//also for logging
		size_t last_slash_id = name.find_last_of("/");
		if (last_slash_id == std::string::npos)
			last_slash_id = 0;
		else
			++last_slash_id;

		//return the name of the object, that is the correponding pointcloud name

		return name.substr(last_slash_id, name.find_last_of('.')
                       - last_slash_id).c_str();
	  }

	keypoints.clear();

	return "";
}
*/
void NBVTree::project_centroid( PointCloud::Ptr input_cloud, std::vector<Eigen::Vector4f>& centroids)
{
	std::vector<pcl::PointIndices> cluster_indices;
	project_centroid(input_cloud, centroids, cluster_indices);
}


void NBVTree::project_centroid( PointCloud::Ptr input_cloud, 
								std::vector<Eigen::Vector4f>& centroids, 
								std::vector<pcl::PointIndices>& cluster_indices)
{

    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (input_cloud);

    //std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.1); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(input_cloud);
    ec.extract (cluster_indices);
    
    if(DEBUG)
    	ROS_INFO("size of clusters: %d",cluster_indices.size());

   	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	   	      {
	   	         pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
	   	        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	   	             cloud_cluster->points.push_back (input_cloud->points[*pit]);

	   	       // Now moving every cluster to the center of the view sphere
	   	       // compute centroid for every cluster
	   	       Eigen::Vector4f min, max,centroid;
	   	       pcl::getMinMax3D(*cloud_cluster, min, max);
	   	       centroid = (max + min)/2;
	   	       //centroid projection to z=0
	   	       //centroid.z() = 0;

	   	       centroids.push_back(centroid);
	   	      }


}

void 
NBVTree::get_cluster_centroid(PointCloud::Ptr cloud, Eigen::Vector3d &centroid, std::vector<Eigen::Vector4f>& centroids, std::vector<pcl::PointIndices>& cluster_indices, PointCloud::Ptr& cloud_cluster_new)
{
        // now re-clustering the point cloud
        pcl::search::KdTree<PointT>::Ptr tree_new (new pcl::search::KdTree<PointT>);
        tree_new->setInputCloud (cloud);

        cluster_indices.clear();

        pcl::EuclideanClusterExtraction<PointT> ec_new;
        ec_new.setClusterTolerance (0.1); // 2cm
        ec_new.setMinClusterSize (100);
        ec_new.setMaxClusterSize (25000);
        ec_new.setSearchMethod (tree_new);
        ec_new.setInputCloud(cloud);
        ec_new.extract (cluster_indices);

        if(DEBUG){
        	ROS_INFO("size of clusters: %d",cluster_indices.size());
			ROS_INFO("Extracting clusters for reoriented view");
		}

        centroids.clear();

        PointCloud::Ptr cloud_new(new PointCloud);

        bool flag = false;
        int j = 0;
	   	for (std::vector<pcl::PointIndices>::const_iterator it_n = cluster_indices.begin (); it_n != cluster_indices.end (); ++it_n)
	   	      {
	   		cloud_new.reset(new PointCloud);
	   	        for (std::vector<int>::const_iterator pit_n = it_n->indices.begin (); pit_n != it_n->indices.end (); pit_n++){
	   	        	cloud_new->points.push_back (cloud->points[*pit_n]);
	   	        }


    	   	       Eigen::Vector4f min_n, max_n, centroid_new;
    	   	       pcl::getMinMax3D(*cloud_new, min_n, max_n);

    	   	       centroid_new = (max_n + min_n)/2;

    	   	       // now create a temporary search point and assing the centroid to it
    	   	       pcl::PointXYZ searchPoint_n;

    	   	       searchPoint_n.x = centroid.x();
    	   	       searchPoint_n.y = centroid.y();
    	   	       searchPoint_n.z = centroid.z();

    	   	       centroids.push_back(centroid_new);


    	   	       pcl::search::KdTree<PointT>::Ptr kdtree_temp (new pcl::search::KdTree<PointT>);

    	   	       kdtree_temp->setInputCloud (cloud_new);

    	   	       float radius_n = 0.15;

    		   	    std::vector<int> pointIdxRadiusSearch;
    		   	    std::vector<float> pointRadiusSquaredDistance;

    		   	    if ( kdtree_temp->radiusSearch (searchPoint_n, radius_n, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    		   	    {
    	   	    	   if(!flag)
    	   	    	   {
    	   	    		 //ROS_INFO("Processing the newly found cluster");
    	   	    		 for (size_t i = 0; i < cloud_new->points.size (); ++i)
    	   	    			 {
    	   	    			cloud_new->points[i].x -= centroid_new.x();
    	   	    			cloud_new->points[i].y -= centroid_new.y();
    	   	    			cloud_new->points[i].z -= centroid_new.z();
    	   	    			 }

    	   	    		centroid.x() = (centroid_new.x());// + centroid.x())/2;
    	   	    		centroid.y() = (centroid_new.y());// + centroid.y())/2;

       	   	    		 flag = true;
       	   	    		 copyPointCloud(*cloud_new,*cloud_cluster_new);

    	   	    	   }
    	   	       }
    	   	       // clear current indices
    	   	       j++;
	   	      }
	}

void NBVTree::pose_refinement(PointCloud::Ptr input_cloud, int viewpoint, int hypothesis,Eigen::Matrix3f& rotation){

	// find the corresponding viewpoint
	// use the score map
	MatrixXf view_correspondence(30,6);
	load_matrix(view_correspondence,correspondence_file,30,6);


	// now getting the corresponding view
	int view = view_correspondence(viewpoint,hypothesis);

	int hyp[]={0,60,120,180,240,300};

	// reading the appropriate pointcloud from the data base
	std::stringstream new_name;
	new_name<<clouds_directory<<"/"<<match_name<<"_views/"<<match_name<<"_yaw"<<hyp[hypothesis]<<"_vp_"<<view<<".pcd";

	ROS_INFO("Performing Fine pose refinement");
	// read the point cloud from disk

	PointCloud::Ptr target_cloud(new PointCloud);
	pcl::PCDReader reader;
	reader.read (new_name.str().c_str(), *target_cloud);

	// perform simple ICP for now
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(0.05);
	icp.setTransformationEpsilon(1e-8);
	icp.setMaximumIterations(50);
	icp.setEuclideanFitnessEpsilon(1);
	icp.setInputCloud(input_cloud);
	icp.setInputTarget(target_cloud);


	pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*Final);

	Eigen::Matrix4f	transform = icp.getFinalTransformation();

	// get rotation and translation
	rotation = transform.block<3,3>(0,0);
	Eigen::Vector3f translation = transform.block<3,1>(0,3);
}

void NBVTree::align_clouds(PointCloud::Ptr& input_cloud, PointCloud::Ptr target_cloud, PointCloud::Ptr& final_cloud){

	// find the corresponding viewpoint
	// perform simple ICP for now
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(0.05);
	icp.setTransformationEpsilon(1e-8);
	icp.setMaximumIterations(50);
	icp.setEuclideanFitnessEpsilon(1);
	icp.setInputCloud(input_cloud);
	icp.setInputTarget(target_cloud);

	final_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*final_cloud);
	*final_cloud += *target_cloud;

}


