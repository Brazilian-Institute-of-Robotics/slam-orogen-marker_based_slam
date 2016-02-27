require 'vizkit'
require 'orocos'
include Orocos

g_relative_poses = Hash.new
g_camera_pose = Hash.new
Orocos.initialize

Orocos.run do
   
   #Orocos.log_all 
   view3d = Vizkit.vizkit3d_widget

   ##########################################
   ###   Get Task context of Enviroments  ###
   ##########################################
   #camera = TaskContext.get ''
   #single_marker_detector = Orocos.name_service.get 'single_marker_detector'
   #single_marker_detector = Orocos.name_service.get_provides 'aruco::SingleMarkerDetector'
   mb_mapping = Orocos.name_service.get 'marker_based_slam'    

   ##########################################
   ###     Apply Configuration Files      ###
   ##########################################
   #Orocos.apply_conf_file(camera, './ConfigFiles/camera_aravis.yml', ['default','aravis_basler'])
   #Orocos.apply_conf_file(single_marker_detector, './ConfigFiles/singlemarkerdetector.yml', ['default','aravis_basler'])
    

   ##########################################
   ###        Configure Tasks             ###
   ##########################################

   #camera.configure
   #single_marker_detector.configure
   #mb_mapping.configure
    
   ##########################################
   ###              Start Tasks           ###
   ##########################################
   #mb_mapping.start
   #camera.start
   #single_marker_detector.start

   ##########################################
   ###     Connect Ports                  ###
   ##########################################
   #camera.frame.connect_to single_marker_detector.image
   #single_marker_detector.marker_pose.connect_to mb_mapping.marker_pose

    ################################
    ####### 3D Visualization #######
    ################################
    

    
    # get all the relative poses    
    mb_mapping.relative_poses.connect_to do |samples,_|
        
        # create a rigid body state for all relative poses
        for i in 0...samples.length
	        rbs = Types::Base::Samples::RigidBodyState.new
 
            #avoid duplicate the vizkit rbs
	        if !(g_relative_poses.has_key?([samples[i].pair1, samples[i].pair2])) 
	             g_relative_poses[[samples[i].pair1, samples[i].pair2]] = Vizkit.default_loader.RigidBodyStateVisualization
	        end
                
            #set configurations for the vizkit rbs (size and name)
            g_relative_poses[[samples[i].pair1, samples[i].pair2]].sphereSize = 0.20	
            g_relative_poses[[samples[i].pair1, samples[i].pair2]].size       = 0.40	
            g_relative_poses[[samples[i].pair1, samples[i].pair2]].frame      = samples[i].pair2.to_s
                
            #update the position and orientation
            rbs.position = samples[i].rbs.position
            rbs.orientation = samples[i].rbs.orientation

            g_relative_poses[[samples[i].pair1, samples[i].pair2]].updateRigidBodyState(rbs)
        end

        # 
	    g_relative_poses.each {|key, value|
    	    flag = false
            for i in 0...samples.length
		        #unable the vizkit plugin and clean the hash pair
		        if (key == [samples[i].pair1, samples[i].pair2])
		            flag = true
		            break
		        end
	        end
	        if flag == false
	            g_relative_poses[key].enabled = false
	            g_relative_poses.delete(key)		
	        end 
	    }                
    end

    mb_mapping.camera_pose.connect_to do |samples2,_|
        for i in 0...samples2.length
       
	        if !(g_camera_pose.has_key?(samples2[i].id)) 
                g_camera_pose[samples2[i].id] = Vizkit.default_loader.RigidBodyStateVisualization
            end
	
            g_camera_pose[samples2[i].id].sphereSize = 0.4
            g_camera_pose[samples2[i].id].size = 0.8
            g_camera_pose[samples2[i].id].frame = samples2[i].id.to_s

            rbs = Types::Base::Samples::RigidBodyState.new
            
            rbs.position = samples2[i].rbs.position
            rbs.orientation = samples2[i].rbs.orientation 

            g_camera_pose[samples2[i].id].updateRigidBodyState(rbs) 	
	    end
		
	    g_camera_pose.each {|key, value|
	        flag = false
            for i in 0...samples2.length
		        #unable the vizkit plugin and clean the hash pair
		        if (key == samples2[i].id)
		        flag = true
		        break
		        end
	        end
	        if flag == false
	            g_camera_pose[key].enabled = false
	            g_camera_pose.delete(key)		
	        end 
	    }                
    end

    #Vizkit.display single_marker_detector.augmented_image
    #Vizkit.display single_marker_detector.threshoulded_image
    view3d.show
    view3d.windowTitle = "Map"
    Vizkit.exec
end
