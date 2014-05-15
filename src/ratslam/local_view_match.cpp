/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "local_view_match.h"
#include "../utils/utils.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <iomanip>
using namespace std;
#include <boost/foreach.hpp>
#include <algorithm>

#include <stdio.h>

//Start My Changes****************************************
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/core/core.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//END My Changes****************************************


namespace ratslam
{



LocalViewMatch::LocalViewMatch(ptree settings)
{
  get_setting_from_ptree(VT_MIN_PATCH_NORMALISATION_STD, settings, "vt_min_patch_normalisation_std", (double)0);
  get_setting_from_ptree(VT_PATCH_NORMALISATION, settings, "vt_patch_normalise", 0);
  get_setting_from_ptree(VT_NORMALISATION, settings, "vt_normalisation", (double) 0);
  get_setting_from_ptree(VT_SHIFT_MATCH, settings, "vt_shift_match", 25);
  get_setting_from_ptree(VT_STEP_MATCH, settings, "vt_step_match", 5);
  get_setting_from_ptree(VT_PANORAMIC, settings, "vt_panoramic", 0);
 
  get_setting_from_ptree(VT_MATCH_THRESHOLD, settings, "vt_match_threshold", 0.03);
  get_setting_from_ptree(TEMPLATE_X_SIZE, settings, "template_x_size", 1);
  get_setting_from_ptree(TEMPLATE_Y_SIZE, settings, "template_y_size", 1);
  get_setting_from_ptree(IMAGE_VT_X_RANGE_MIN, settings, "image_crop_x_min", 0);
  get_setting_from_ptree(IMAGE_VT_X_RANGE_MAX, settings, "image_crop_x_max", -1);
  get_setting_from_ptree(IMAGE_VT_Y_RANGE_MIN, settings, "image_crop_y_min", 0);
  get_setting_from_ptree(IMAGE_VT_Y_RANGE_MAX, settings, "image_crop_y_max", -1);

  TEMPLATE_SIZE = TEMPLATE_X_SIZE * TEMPLATE_Y_SIZE;

  templates.reserve(10000);

  current_view.resize(TEMPLATE_SIZE);

  current_vt = 0;
  prev_vt = 0;
}


LocalViewMatch::~LocalViewMatch()
{

}

void LocalViewMatch::on_image(const unsigned char * view_rgb, bool greyscale, unsigned int image_width, unsigned int image_height)
{
  if (view_rgb == NULL) //view_rgb is the image revieved from the camera
    return;

  IMAGE_WIDTH = image_width;
  IMAGE_HEIGHT = image_height;

  if (IMAGE_VT_X_RANGE_MAX == -1)
    IMAGE_VT_X_RANGE_MAX = IMAGE_WIDTH;
  if (IMAGE_VT_Y_RANGE_MAX == -1)
    IMAGE_VT_Y_RANGE_MAX = IMAGE_HEIGHT;

  this->view_rgb = view_rgb;
  this->greyscale = greyscale;
  
  convert_view_to_view_template(greyscale); //This now adds data to the image and current_descriptors variables
  prev_vt = get_current_vt();
  unsigned int vt_match_id;
  compare2(vt_error, vt_match_id);
  if (vt_error <= VT_MATCH_THRESHOLD) 
  {
    set_current_vt((int)vt_match_id);
    cout << "VTM[" << setw(4) << get_current_vt() << "] " << endl;
    cout.flush();
    //ROS_FATAL_STREAM("--------------------------familiar view " << vt_match_id);
    ROS_FATAL_STREAM(" vt error "<< vt_error << " ---VT_ID " <<vt_match_id<<"---------------familiar view ");
   
  }
  else
  {
    vt_relative_rad = 0;
    set_current_vt(create_template());
    cout << "VTN[" << setw(4) << get_current_vt() << "] " << endl;
    cout.flush();
    //ROS_FATAL_STREAM("non-familiar view " << templates.size() << " Nearly " << vt_match_id );
    ROS_FATAL_STREAM(" vt error "<< vt_error << " ---VT_ID " <<vt_match_id<<"---------------non-familiar view ");
   
  }

}


void LocalViewMatch::clip_view_x_y(int &x, int &y)
{
  if (x < 0)
    x = 0;
  else if (x > TEMPLATE_X_SIZE - 1)
    x = TEMPLATE_X_SIZE - 1;

  if (y < 0)
    y = 0;
  else if (y > TEMPLATE_Y_SIZE - 1)
    y = TEMPLATE_Y_SIZE - 1;

}

void LocalViewMatch::convert_view_to_view_template(bool grayscale)
{


  int data_next = 0;
  int sub_range_x = IMAGE_VT_X_RANGE_MAX - IMAGE_VT_X_RANGE_MIN;
  int sub_range_y = IMAGE_VT_Y_RANGE_MAX - IMAGE_VT_Y_RANGE_MIN;
  int x_block_size = sub_range_x / TEMPLATE_X_SIZE;
  int y_block_size = sub_range_y / TEMPLATE_Y_SIZE;
  int pos;

  for (unsigned int i; i < current_view.size(); i++)
    current_view[i] = 0;

  if (grayscale)  // This section cycles through the image, and cycles throuh individual sub blocks(as specified in the config) then averages the pixel values o give a visual template
  {
    for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block += y_block_size, y_block_count++)
    {
      for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block += x_block_size, x_block_count++)
      {
        for (int x = x_block; x < (x_block + x_block_size); x++)
        {
          for (int y = y_block; y < (y_block + y_block_size); y++)
          {
            pos = (x + y * IMAGE_WIDTH);
            current_view[data_next] += (double)(view_rgb[pos]);
            //ROS_FATAL_STREAM("Pixel " << current_view[data_next] );
  
          }
        }
        //ROS_FATAL_STREAM("Pixel " << current_view[data_next] );
        current_view[data_next] /= (255.0);
        current_view[data_next] /= (x_block_size * y_block_size);
        //ROS_FATAL_STREAM("Pixel normed" << current_view[data_next] );
        data_next++;
        
  
      }
    }
  }
  else //this does the same for colour images
  {
    for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block +=
        y_block_size, y_block_count++)
    {
      for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block +=
          x_block_size, x_block_count++)
      {
        for (int x = x_block; x < (x_block + x_block_size); x++)
        {
          for (int y = y_block; y < (y_block + y_block_size); y++)
          {
            pos = (x + y * IMAGE_WIDTH) * 3;
            current_view[data_next] += ((double)(view_rgb[pos]) + (double)(view_rgb[pos + 1])
                + (double)(view_rgb[pos + 2]));
          }
        }
        current_view[data_next] /= (255.0 * 3.0);
        current_view[data_next] /= (x_block_size * y_block_size);

        data_next++;
      }
    }
  }

  if (VT_NORMALISATION > 0)
  {
    double avg_value = 0;

    for (unsigned int i = 0; i < current_view.size(); i++)
    {
      avg_value += current_view[i];
    }

    avg_value /= current_view.size();

    for (unsigned int i = 0; i < current_view.size(); i++)
    {
      current_view[i] = std::max(0.0, std::min(current_view[i] * VT_NORMALISATION / avg_value, 1.0));
    }
  }

  // now do patch normalisation
  // +- patch size on the pixel, ie 4 will give a 9x9
  if (VT_PATCH_NORMALISATION > 0)
  {
    int patch_size = VT_PATCH_NORMALISATION;
    int patch_total = (patch_size * 2 + 1) * (patch_size * 2 + 1);
    double patch_sum;
    double patch_mean;
    double patch_std;
    int patch_x_clip;
    int patch_y_clip;

    // first make a copy of the view
    std::vector<double> current_view_copy;
    current_view_copy.resize(current_view.size());
    for (unsigned int i = 0; i < current_view.size(); i++)
      current_view_copy[i] = current_view[i];

    // this code could be significantly optimimised ....
    for (int x = 0; x < TEMPLATE_X_SIZE; x++)
    {
      for (int y = 0; y < TEMPLATE_Y_SIZE; y++)
      {
        patch_sum = 0;
        for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
        {
          for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
          {
            patch_x_clip = patch_x;
            patch_y_clip = patch_y;
            clip_view_x_y(patch_x_clip, patch_y_clip);

            patch_sum += current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE];
          }
        }
        patch_mean = patch_sum / patch_total;

        patch_sum = 0;
        for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
        {
          for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
          {
            patch_x_clip = patch_x;
            patch_y_clip = patch_y;
            clip_view_x_y(patch_x_clip, patch_y_clip);

            patch_sum += ((current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean)
                * (current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean));
          }
        }

        patch_std = sqrt(patch_sum / patch_total);

        if (patch_std < VT_MIN_PATCH_NORMALISATION_STD)
          current_view[x + y * TEMPLATE_X_SIZE] = 0.5;
        else {
          current_view[x + y * TEMPLATE_X_SIZE] = max((double) 0, min(1.0, (((current_view_copy[x + y * TEMPLATE_X_SIZE] - patch_mean) / patch_std) + 3.0)/6.0 ));
        }
      }
    }
  }

  double sum = 0;

  // find the mean of the data
  for (int i = 0; i < current_view.size(); i++)
    sum += current_view[i];

  current_mean = sum/current_view.size();
 

  
  //Start My Changes****************************************
  // Find Features in image
  //image = cv::Mat(TEMPLATE_Y_SIZE,TEMPLATE_X_SIZE, 16);
  current_keypoints.clear();
  current_descriptors.release();
  current_image.release();
  current_view2.clear();


  for (int i = 0; i < current_view.size(); i++){
    current_view2.push_back(current_view[i]);
    current_view2.push_back(current_view[i]);
    current_view2.push_back(current_view[i]);
  }
  current_image = cv::Mat(current_view2,true); 
  //ROS_FATAL_STREAM("Just copied image to mat dimensions are : " << image.rows << "  " << image.cols );
  //ROS_FATAL_STREAM("the mat type before is : "<< image.type() );
  current_image = current_image.reshape(3,TEMPLATE_Y_SIZE); 
  //ROS_FATAL_STREAM("Here is the image Mat"<<  current_image);
  current_image.convertTo(current_image,16,256);

  //ROS_FATAL_STREAM("Here is the image Mat"<<  current_image);
  //ROS_FATAL_STREAM("converted image to right format");
  //ROS_FATAL_STREAM("the mat depth is : "<< current_image.depth() );
  //ROS_FATAL_STREAM("the mat type is : "<< current_image.type() );
  //ROS_FATAL_STREAM("the no of channels is : "<< image.channels());
  //ROS_FATAL_STREAM("Attempting to reshape to this many rows : " << TEMPLATE_Y_SIZE);
  //ROS_FATAL_STREAM("Just reshaped the image dimensions are now : "<<  current_image.rows << "  " << current_image.cols);
  //ROS_FATAL_STREAM("Greyscale true : "<<  this->greyscale);
  //cv::imshow("image",image);
  //cv::waitKey(0);
  //ROS_FATAL_STREAM("Here is the image Mat"<<  image);
  
  detector.detect(current_image, current_keypoints);
  //ROS_FATAL_STREAM("Just found keypoints, found " << current_keypoints.size());
  //ROS_FATAL_STREAM("Before descriptors are found they are " << current_descriptors.size());
  cv::Mat current_image2;
  
  
  extractor.compute(current_image, current_keypoints, current_descriptors);
  ROS_FATAL_STREAM("No of descriptors is : " << current_descriptors.rows);
  cv::drawKeypoints(current_image, current_keypoints,current_image2,cv::Scalar::all(-1),4);
  
  cv::namedWindow("current_image",CV_WINDOW_NORMAL);
  cv::imshow("current_image", current_image2);
  cv::waitKey(3);
  //cv::waitKey(0);
  //END My Changes****************************************/
  //cv::destroyWindow("image");
  
}

// create and add a visual template to the collection
int LocalViewMatch::create_template()
{
  templates.resize(templates.size() + 1);
  VisualTemplate * new_template = &(*(templates.end() - 1));

  new_template->id = templates.size() - 1;
  double * data_ptr = &current_view[0];
  new_template->data.reserve(TEMPLATE_SIZE);
  for (int i = 0; i < TEMPLATE_SIZE; i++)
    new_template->data.push_back(*(data_ptr++));

  new_template->mean = current_mean;
  new_template->keypoints = current_keypoints;
  new_template->descriptors = current_descriptors;
  new_template->image = current_image;

  //ROS_FATAL_STREAM("Successfully made new template");
    
  return templates.size() - 1;
}

// compare a visual template to all the stored templates, allowing for 
// slen pixel shifts in each direction
// returns the matching template and the MSE
void LocalViewMatch::compare(double &vt_err, unsigned int &vt_match_id)
{
  if (templates.size() == 0)
  {
    vt_err = DBL_MAX;
    vt_error = vt_err;
    return;
  }
  
  double *data = &current_view[0];
  double mindiff, cdiff;
  mindiff = DBL_MAX;

  vt_err = DBL_MAX;
  int min_template = 0;

  double *template_ptr;
  double *column_ptr;
  double *template_row_ptr;
  double *column_row_ptr;
  double *template_start_ptr;
  double *column_start_ptr;
  int row_size;
  int sub_row_size;
  double *column_end_ptr;
  VisualTemplate vt;
  int min_offset;

  int offset;
  double epsilon = 0.005;
  

  if (VT_PANORAMIC)// Ill never use this with the Qbo
  {

  	BOOST_FOREACH(vt, templates)
  	{

    	if (abs(current_mean - vt.mean) > VT_MATCH_THRESHOLD + epsilon)
    	  continue;

    	// for each vt try matching the view at different offsets
    	// try to fast break based on error already great than previous errors
    	// handles 2d images shifting only in the x direction
    	// note I haven't tested on a 1d yet.
    	for (offset = 0; offset < TEMPLATE_X_SIZE; offset += VT_STEP_MATCH)
    	{
    	  cdiff = 0;
    	  template_start_ptr = &vt.data[0] + offset; // & mean the pointer to the address of vt.data[0] &variable means what is the address of variable, *variable would mean what is the at the address stored in varialbe
    	  column_start_ptr = &data[0];
    	  row_size = TEMPLATE_X_SIZE;
    	  column_end_ptr = &data[0] + TEMPLATE_SIZE - offset;
    	  sub_row_size = TEMPLATE_X_SIZE - offset;

    	  // do from offset to end
    	  for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
    	  {
      		for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
      		{
      		  cdiff += abs(*column_ptr - *template_ptr);
      		}

      		// fast breaks
      		if (cdiff > mindiff)
      		  break;
    	  }

    	  // do from start to offset
    	  template_start_ptr = &vt.data[0];
    	  column_start_ptr = &data[0] + TEMPLATE_X_SIZE - offset;
    	  row_size = TEMPLATE_X_SIZE;
    	  column_end_ptr = &data[0] + TEMPLATE_SIZE;
    	  sub_row_size = offset;
    	  for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
    	  {
    		for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
    		{
    		  cdiff += abs(*column_ptr - *template_ptr);
    		}

    		// fast breaks
    		if (cdiff > mindiff)
    		  break;
    	  }


    	  if (cdiff < mindiff)
    	  {
    		mindiff = cdiff;
    		min_template = vt.id;
    		min_offset = offset;
    	  }
    	}

  	}

  	vt_relative_rad = (double) min_offset/TEMPLATE_X_SIZE * 2.0 * M_PI;
  	if (vt_relative_rad > M_PI)
  	vt_relative_rad = vt_relative_rad - 2.0 * M_PI;
  	vt_err = mindiff / (double) TEMPLATE_SIZE;
  	vt_match_id = min_template;

  	vt_error = vt_err;

  } else {

  	BOOST_FOREACH(vt, templates)
  	{

    	if (abs(current_mean - vt.mean) > VT_MATCH_THRESHOLD + epsilon)
    	  continue; // skips this vt if the means are too different

    	// for each vt try matching the view at different offsets
    	// try to fast break based on error already great than previous errors
    	// handles 2d images shifting only in the x direction
    	// note I haven't tested on a 1d yet.
    	for (offset = 0; offset < VT_SHIFT_MATCH*2+1; offset += VT_STEP_MATCH)
    	{
    	  cdiff = 0;
    	  template_start_ptr = &vt.data[0] + offset;
    	  column_start_ptr = &data[0] + VT_SHIFT_MATCH;
    	  row_size = TEMPLATE_X_SIZE;
    	  column_end_ptr = &data[0] + TEMPLATE_SIZE - VT_SHIFT_MATCH;
    	  sub_row_size = TEMPLATE_X_SIZE - 2*VT_SHIFT_MATCH;

    	  for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
    	  {
      		for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
      		{
      		  cdiff += abs(*column_ptr - *template_ptr);
      		}

      		// fast breaks
      		if (cdiff > mindiff)
      		  break;
    	  }

    	  if (cdiff < mindiff)
    	  {
      		mindiff = cdiff;
      		min_template = vt.id;
      		min_offset = 0;
    	  }
    	}

  	}

  	vt_relative_rad = 0;
  	vt_err = mindiff / (double)(TEMPLATE_SIZE - 2 * VT_SHIFT_MATCH * TEMPLATE_Y_SIZE);
  	vt_match_id = min_template;

  	vt_error = vt_err;

  }
}


void LocalViewMatch::compare2(double &vt_err, unsigned int &vt_match_id)
{
  
  if (templates.size() == 0)
  {
    //vt_err = DBL_MAX;
    vt_error = 0;//vt_err;
    return;
  }
  vt_error = 0;    
  vt_match_id = 0;
  double *data = &current_view[0];
  double mindiff, cdiff;
  mindiff = DBL_MAX;

  //vt_err = DBL_MAX;
  int min_template = 0;

  double *template_ptr;
  double *column_ptr;
  double *template_row_ptr;
  double *column_row_ptr;
  double *template_start_ptr;
  double *column_start_ptr;
  int row_size;
  int sub_row_size;
  double *column_end_ptr;
  ratslam::VisualTemplate vt;
  int min_offset;

  int offset;
  double epsilon = 0.05;
  
  int max_matches= 0;
  // Debugging
  int total_comparisons = 0;
  int mean_breaks = 0;
  int descriptor_breaks = 0;
  //************************/
  ros::Time timer;
  ros::Time starttime = timer.now();

  BOOST_FOREACH(vt, templates) // reverse search may be optimal
  {
    //vt_error = 0;
     
    total_comparisons++;  
    if (abs(current_mean - vt.mean) > epsilon){
      //ROS_FATAL_STREAM("mean diff "<<(current_mean - vt.mean));
      mean_breaks++;
      continue; // skips this vt if the means are too different
    }
    if (vt.descriptors.rows < -0.5*VT_MATCH_THRESHOLD ){
      descriptor_breaks++;
      continue; // skips this vt if there are not enough descriptors
    }
    if (vt.descriptors.rows > 1.5*current_descriptors.rows ){
      descriptor_breaks++;
      continue; // skips this vt if there are too many descriptors
    }

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    //cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20,10,2));
    std::vector< cv::DMatch > matches; 
    matcher.match( current_descriptors, vt.descriptors, matches );
    if (matches.size() == 0){
      continue;
    }

    std::vector< cv::DMatch > good_matches;
      
    if (current_descriptors.rows > 0){
            
      for( int i = 0; i < current_descriptors.rows; i++ )
      {    
        if( matches[i].distance <= 5)
        { 
          good_matches.push_back( matches[i]); }
      }
    }
    //-- Draw only "good" matches
    cv::Mat img_matches;
    if (good_matches.size() > 0)
    {
      drawMatches( current_image, current_keypoints, vt.image, vt.keypoints, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), cv::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    }

    if (good_matches.size() > max_matches){
      max_matches=good_matches.size();
      min_template = vt.id;
      vt_match_id = min_template ;  
      vt_error = -1*max_matches;
      cv::namedWindow("Good Matches",CV_WINDOW_NORMAL);
      cv::imshow( "Good Matches", img_matches );
      cv::waitKey(3);
      //ROS_FATAL_STREAM("Matches " << matches.size() );
      if (max_matches >= VT_MATCH_THRESHOLD){
        //if (abs(current_mean - vt.mean)>max_match_mean_diff)
          //max_match_mean_diff =   abs(current_mean - vt.mean);
        continue;
      }
      if (max_matches >= 0.9*vt.descriptors.rows){
        
        continue;
      }
     
    }     
    //vt_match_id = min_template;
    
    //ROS_FATAL_STREAM("Max Match Mean diff "<<max_match_mean_diff);
  }
  ros::Duration duration = timer.now()-starttime;
  //double durationsecs  = duration.sec + 0.000000001*duration.nsec;
  //ROS_FATAL_STREAM("Time diff "<<(duration.toSec()) << " for " << templates.size() <<" templates");
  //ROS_FATAL_STREAM("Time/template ~= " << ((duration.toSec())/templates.size())) ;
  ROS_FATAL_STREAM("Time/Maxtime (0.2) ~= " << ((duration.toSec())/0.2)) ;
  
  
  //ROS_FATAL_STREAM("total_comparisons " <<total_comparisons<<" total templates "<<templates.size()<< " ratio " <<float(total_comparisons/templates.size()));
    
}
}


