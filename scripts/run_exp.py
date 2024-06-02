import roslaunch
import rosparam
import rospy
import random
import pandas as pd
import datetime
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
result_path = dir_path + "/exp_results"
config_path = dir_path + "/config"

# ns = "~"
class random_trials():
    def __init__(self) -> None:
        self.package = 'src_loc'
        self.mts_trej_exec = 'MTS_trej_node'
        self.sensor_node_exec = 'sensor_node'
        self.mts_algo_node_exec = 'MTS_algo_node'
    
        self.package = 'src_loc'
        self.mts_trej_exec = 'MTS_trej_node'
        self.sensor_node_exec = 'sensor_node'
        self.mts_algo_node_exec = 'MTS_algo_node'

        self.vars_no = 5
        self.src_loc_no = 10
        self.vars = [[50.0, 75.0], [75.0, 75.0], [100.0, 75.0], [75.0, 50.0], [75.0, 100.0]]
        # self.vars = [[50.0, 75.0], [75.0, 75.0], [75.0, 50.0]]
        self.source_locs = [[[0.0, 0.0] for _ in range(self.src_loc_no)] for _ in range(self.vars_no)]
        self.m_list = [[0,1,1], [1,1,0], [0,0,1]]
        self.h_max = [0.0021, 0.000091 , 0.0019 ] 


    def gen_ini_conds(self, field):
        
        # initiate seeds, for reproduceble randomness
        random.seed(1)
        generator1 = random.Random()
        generator2 = random.Random()
        generator1.seed(1)
        generator2.seed(2)


        if (field == "arva"):
            self.source_locs = [[0.0, 0.0] for _ in range(self.src_loc_no)] 
            # generate set of random source points for each of variance in field. 
            for j in range(self.src_loc_no):
                src_x = random.uniform(0.8 * 50, 1 * 50)
                src_y = random.uniform(0.8 * 50, 1 * 50)

                self.source_locs[j][0] = src_x
                self.source_locs[j][1] = src_y

        else:
            # generate set of random source points for each of variance in field. 
            for i in range(self.vars_no):
                sig_x = self.vars[i][0]
                sig_y = self.vars[i][1]
                print(f"sig_x: {sig_x} sig_y: {sig_y}")
                # distribution_sig_x = random.uniform(1.5 * sig_x, 3 * sig_x)
                # distribution_sig_y = random.uniform(1.5 * sig_y, 3 * sig_y)
                for j in range(self.src_loc_no):
                    src_x = random.uniform(0.8 * sig_x, 1.5 * sig_x)
                    src_y = random.uniform(0.8 * sig_y, 1.5 * sig_y)

                    self.source_locs[i][j][0] = src_x
                    self.source_locs[i][j][1] = src_y


    
    def save_ini_conds(self):

        current_datetime = datetime.datetime.now()
        formatted_datetime = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
        if (field == 'arva'):
            ini_cond_fn = f"/home/gowda/catkin_ws/src/src_loc/exp_results/arva_ini_conds_{formatted_datetime}.csv"
            df_init_conds = pd.DataFrame(columns=[
                                            "m",
                                            "pos_s_x",
                                            "pos_s_y",
                                            "psi_0",
            ])
            for m in range(self.m_list):
                for j in range(self.src_loc_no):
                    pos_x_s = self.source_locs[j][0]
                    pos_y_s = self.source_locs[j][1]
                    for psi_0 in range(0, 360, 30):
                        ini_cond_data = {
                                        "m" : m,
                                        "pos_s_x" : pos_x_s,
                                        "pos_s_y" : pos_y_s,
                                        "psi_0": psi_0
                        }
                        df_init_conds = df_init_conds.append(ini_cond_data, ignore_index=True)
                        df_init_conds.to_csv(ini_cond_fn, index=False)
        else: 
            ini_cond_fn = f"/home/gowda/catkin_ws/src/src_loc/exp_results/ini_conds_{formatted_datetime}.csv"
            df_init_conds = pd.DataFrame(columns=[
                                            "sig_x",
                                            "sig_y",
                                            "pos_s_x",
                                            "pos_s_y",
                                            "psi_0",
            ])
            for i in range(self.vars_no):
                sig_x = self.vars[i][0]
                sig_y = self.vars[i][1]
                for j in range(self.src_loc_no):
                    pos_x_s = self.source_locs[i][j][0]
                    pos_y_s = self.source_locs[i][j][1]
                    for psi_0 in range(0, 360, 30):
                        ini_cond_data = {
                                        "sig_x" : sig_x,
                                        "sig_y" : sig_y,
                                        "pos_s_x" : pos_x_s,
                                        "pos_s_y" : pos_y_s,
                                        "psi_0": psi_0
                        }
                        df_init_conds = df_init_conds.append(ini_cond_data, ignore_index=True)
                        df_init_conds.to_csv(ini_cond_fn, index=False)
    
    def setup_exp(self, field):


        # set initilization parameters
        if field == 'arva':
            params = rosparam.load_file('/home/gowda/catkin_ws/src/src_loc/config/initialize_arva.yaml')
            sensor_params = rosparam.load_file('/home/gowda/catkin_ws/src/src_loc/config/sensor_map_arva.yaml', "grid_map_visualization")
        else:
            params = rosparam.load_file('/home/gowda/catkin_ws/src/src_loc/config/initialize.yaml')
            sensor_params = rosparam.load_file('/home/gowda/catkin_ws/src/src_loc/config/sensor_map.yaml', "grid_map_visualization")

        for param, ns in params:
            rosparam.upload_params(ns, param)

        for sensor_param, ns in sensor_params:
            rosparam.upload_params(ns, sensor_param)

        # launch required nodes
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()    
        # grid map visualization node
        grid_map_node = roslaunch.core.Node(package="grid_map_visualization", 
                            node_type="grid_map_visualization",
                            name="grid_map_visualization",
                            output="screen" 
                            )
        self.grid_map_process = self.launch.launch(grid_map_node) 
        
        #  rviz nodes for visualization
        rviz1_node = roslaunch.core.Node( node_type="rviz", 
                            name="rviz_1",
                            package="rviz",
                            args="-d $(find src_loc)/rviz/rviz_mts_test.rviz"
                            )
        self.rviz1_process = self.launch.launch(rviz1_node) 

        rviz2_node = roslaunch.core.Node( node_type="rviz",
                            name="rviz_2",
                            package="rviz",
                            args="-d $(find src_loc)/rviz/rviz_mts_ortho_view.rviz"
                            )
        self.rviz2_process = self.launch.launch(rviz2_node) 

        tf_publish_node = roslaunch.core.Node(node_type="static_transform_publisher",
                            package="tf",
                            name="static_transform_publisher",
                            output="screen",
                            args=" 0 0 0 0 0 0 1 map my_frame 100"
                            )
        self.tf_publish_process = self.launch.launch(tf_publish_node) 

        rospy.loginfo("started ini_params")
        
        # start gazebo and sitl and mavros using launch file
        self.uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid1)
        self.launch_px4_sitl = roslaunch.parent.ROSLaunchParent(self.uuid1 , ["/home/gowda/catkin_ws/src/src_loc/launch/px4_ros.launch"])
        self.launch_px4_sitl.start()
        rospy.sleep(rospy.Duration(20))
        
        # setup trajectory, algo and sensor nodes 
        self.mts_trej_node = roslaunch.core.Node(self.package, self.mts_trej_exec)
        self.sensor_node = roslaunch.core.Node(self.package, self.sensor_node_exec)
        self.mts_algo_node = roslaunch.core.Node(self.package, self.mts_algo_node_exec)

        # launch sensor node 
        # self.process_sensor = self.launch.launch(self.sensor_node)
        rospy.logwarn("Sensor has to be intiated now.") 
    
         
    def run_exp(self, field):
        # initiate node
        rospy.init_node('run_exp_py', anonymous=True)
        
        self.setup_exp(field)

        current_datetime = datetime.datetime.now()
        formatted_datetime = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")

        algo_no = rospy.get_param("/algo_num")
        sensor_noise_no = rospy.get_param("/noise_flag")
        # only for gdts/gamts 
        grad_no = rospy.get_param("/grad_est")
        
        if(sensor_noise_no == 0):
           sensor_noise = "NoNoise" 
        else:
            sensor_noise = "Noise" 

        if(algo_no == 1):
            algo = "MTS"
        elif(algo_no == 2):
            if(grad_no == 1):
                algo = "GAMTS-LSM"
            elif(grad_no ==2):
                algo = "GAMTS-FDM"
        elif(algo_no == 3):
            if(grad_no == 1):
                algo = "GDTS-LSM"
            elif(grad_no ==2):
                algo = "GDTS-FDM"
        elif(algo_no == 4):
            algo = "Sliding-Mode"
        elif(algo_no == 5):
            algo = "zig-zag"
        elif(algo_no == 6):
            algo = "TRM"

        exp_data_fn = f"/home/gowda/catkin_ws/src/src_loc/exp_results/{algo}_{sensor_noise}_exp_data_{formatted_datetime}.csv"


        df = pd.DataFrame(columns= ['var_idx i', 
                                    'src_loc_idx j',
                                    'sig_x',
                                    'sig_y',
                                    'psi_0',
                                    'pos_x_s',
                                    'pos_y_s',
                                    'loc_time']
                        )
        
        # isFirstLoopOfVars = True
        
        for i in range(self.vars_no):
            sig_x = self.vars[i][0]
            sig_y = self.vars[i][1]

            # if (isFirstLoopOfVars):
            #     start_index = 0 # incase failure in a specific case.
            #     isFirstLoopOfVars = 0
            # else :
            start_index = 0 
                
            for j in range(start_index, self.src_loc_no):
                for psi_0 in range(0, 360, 30):
                    print(sig_x, sig_y)
                    rospy.set_param('/sx', sig_x)
                    rospy.set_param('/sy', sig_y)
                    rospy.set_param('/yaw_ini', psi_0)

                    pos_x_s = self.source_locs[i][j][0]
                    pos_y_s = self.source_locs[i][j][1]
                    rospy.set_param('/pos_x_s', pos_x_s)
                    rospy.set_param('/pos_y_s', pos_y_s)
                    print(pos_x_s, pos_y_s)
                    self.process_sensor = self.launch.launch(self.sensor_node)
                    process_mts_trej = self.launch.launch(self.mts_trej_node)
                    process_mts_algo = self.launch.launch(self.mts_algo_node)
                    if [sig_x, sig_y] not in [[75, 100],[100,75]]:
                        if algo_no != 4 and algo_no != 5 and algo_no != 6:
                            while (process_mts_trej.is_alive() and process_mts_algo.is_alive()):
                                pass    
                            process_mts_algo.stop()            
                            process_mts_trej.stop() 
                            self.process_sensor.stop() 
                        else:
                            while (process_mts_trej.is_alive()):
                                pass
                            process_mts_trej.stop() 
                            self.process_sensor.stop() 

                        if rospy.has_param("/loc_time"):
                            loc_time = rospy.get_param("/loc_time")
                            rospy.delete_param("/loc_time")
                            print(f'variance : ({self.vars[i][0]},{self.vars[i][1]}), psi_0 :{psi_0}, loc_time: {loc_time}')
                            data = {'var_idx i': i, 
                                    'src_loc_idx j': j,
                                    'sig_x': sig_x,
                                    'sig_y': sig_y,
                                    'psi_0': psi_0,
                                    'pos_x_s': pos_x_s,
                                    'pos_y_s': pos_y_s,
                                    'loc_time': loc_time
                                    }
                            df = pd.concat([df, pd.DataFrame(data)], ignore_index=True)
                            df.to_csv(exp_data_fn, index=False)
                        else :
                            rospy.logerr("something went wrong")

                        # if loc_time >= 400.0:
                        #     self.launch_px4_sitl.shutdown()        
                        #     self.process_sensor.stop()
                        #     self.rviz1_process.stop()
                        #     self.rviz2_process.stop()
                        #     self.grid_map_process.stop()
                        #     self.tf_publish_process.stop()
                        #     # self.launch.shutdown()
                        #     rospy.logwarn("after shutdown")
                        #     # self.launch_px4_sitl.start()        
                        #     rospy.sleep(rospy.Duration(20))
                        #     rospy.logwarn("after laucnhing")
                        #     rospy.sleep(rospy.Duration(20))
                        #     self.setup_exp(field)
                           
                            

        # self.launch_init_params.shutdown()
        self.launch_px4_sitl.stop()
        self.launch.stop()
        self.process_sensor.stop()
        self.rviz1_process.stop()
        self.rviz2_process.stop()
        self.grid_map_process.stop()
        self.tf_publish_process.stop()

    def run_single_exp(self, field):
        # initiate node
        rospy.init_node('run_exp_py', anonymous=True)

        self.setup_exp(field)

        algo_no = rospy.get_param("/algo_num")

        process_mts_trej = self.launch.launch(self.mts_trej_node)
        process_mts_algo = self.launch.launch(self.mts_algo_node)
        self.process_sensor = self.launch.launch(self.sensor_node)

        if algo_no != 4 and algo_no !=5 and algo_no != 6:
            while (process_mts_trej.is_alive() and process_mts_algo.is_alive()):
                pass    
            process_mts_algo.stop()            
            process_mts_trej.stop() 
            self.process_sensor.stop()
        else:
            while (process_mts_trej.is_alive()):
                pass
            process_mts_trej.stop() 
            self.process_sensor.stop()

        if rospy.has_param("/loc_time"):
            loc_time = rospy.get_param("/loc_time")
            rospy.delete_param("/loc_time")
            print(f"localization time {loc_time}")
        else:
            rospy.logerr("something went wrong")

        self.process_sensor.stop()
        self.launch_px4_sitl.stop()
        self.launch.stop()
        self.rviz1_process.stop()
        self.rviz2_process.stop()
        self.grid_map_process.stop()
        self.tf_publish_process.stop()

    def run_exp_arva(self, field):
        # initiate node
        rospy.init_node('run_exp_py', anonymous=True)

        self.setup_exp(field)

        current_datetime = datetime.datetime.now()
        formatted_datetime = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")

        algo_no = rospy.get_param("/algo_num")
        sensor_noise_no = rospy.get_param("/noise_flag")
        # only for gdts/gamts 
        grad_no = rospy.get_param("/grad_est")
        
        if(sensor_noise_no == 0):
           sensor_noise = "NoNoise" 
        else:
            sensor_noise = "Noise" 

        if(algo_no == 1):
            algo = "MTS"
        elif(algo_no == 2):
            if(grad_no == 1):
                algo = "GAMTS-LSM"
            elif(grad_no ==2):
                algo = "GAMTS-FDM"
        elif(algo_no == 3):
            if(grad_no == 1):
                algo = "GDTS-LSM"
            elif(grad_no ==2):
                algo = "GDTS-FDM"
        elif(algo_no == 4):
            algo = "Sliding-Mode"
        elif(algo_no == 5):
            algo = "zig-zag"
        elif(algo_no == 6):
            algo = "TRM"

        exp_data_fn = f"/home/gowda/catkin_ws/src/src_loc/exp_results/{algo}-ARVA_{sensor_noise}_exp_data_{formatted_datetime}.csv"


        df = pd.DataFrame(columns= ['var_idx i', 
                                    'src_loc_idx j',
                                    'm',
                                    'psi_0',
                                    'pos_x_s',
                                    'pos_y_s',
                                    'loc_time']
                        )
        self.src_loc_no = 8
        self.source_locs = [[[0.0, 0.0] for _ in range(self.src_loc_no)] for _ in range(self.vars_no)]
        self.gen_ini_conds(field)

        for i in range(len(self.m_list)):
            m = self.m_list[i]
            h_max_i = self.h_max[i]
            rospy.set_param('/m', m)
            rospy.set_param('/Qmax', h_max_i)
            for j in range(self.src_loc_no):
                pos_x_s = self.source_locs[j][0]
                pos_y_s = self.source_locs[j][1]
                rospy.set_param('/pos_x_s', pos_x_s)
                rospy.set_param('/pos_y_s', pos_y_s)
                for psi_0 in range(0, 360, 30):
                    rospy.set_param('/yaw_ini', psi_0)
                    process_mts_trej = self.launch.launch(self.mts_trej_node)
                    process_mts_algo = self.launch.launch(self.mts_algo_node)
                    if algo_no != 4 and algo_no !=5 and algo_no != 6:
                        while (process_mts_trej.is_alive() and process_mts_algo.is_alive()):
                            pass    
                        process_mts_algo.stop()            
                        process_mts_trej.stop() 
                    else:
                        while (process_mts_trej.is_alive()):
                            pass
                        process_mts_trej.stop() 

                    if rospy.has_param("/loc_time"):
                        loc_time = rospy.get_param("/loc_time")
                        rospy.delete_param("/loc_time")
                        print(f'm : ({m}), psi_0 :{psi_0}, loc_time: {loc_time}')
                        data = {'var_idx i': i, 
                                'src_loc_idx j': j,
                                'm': m,
                                'psi_0': psi_0,
                                'pos_x_s': pos_x_s,
                                'pos_y_s': pos_y_s,
                                'loc_time': loc_time
                                }
                        df = df.append(data, ignore_index=True)
                        df.to_csv(exp_data_fn, index=False)
                    else :
                        rospy.logerr("something went wrong")

        self.launch_init_params.shutdown()
        self.launch.shutdown()
        self.process_sensor.stop()
        self.rviz1_process.stop()
        self.rviz2_process.stop()
        self.grid_map_process.stop()
        self.tf_publish_process.stop()
        


if __name__ == "__main__":
    exp = random_trials()
    field = "gaussian"
    # field = "arva"

    exp.gen_ini_conds(field)
    # exp.save_ini_conds()    
    # exp.run_exp(field)
    exp.run_single_exp(field)
    # exp.run_exp_arva(field)