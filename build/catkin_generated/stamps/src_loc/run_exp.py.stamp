import roslaunch
import rospy
import random
import pandas as pd
import datetime

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
        self.source_locs = [[[0.0, 0.0] for _ in range(self.src_loc_no)] for _ in range(self.vars_no)]


    def gen_ini_conds(self):
        
        # initiate seeds, for reproduceble randomness
        random.seed(1)
        generator1 = random.Random()
        generator2 = random.Random()
        generator1.seed(1)
        generator2.seed(2)

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
        ini_cond_fn = f"/home/gnc001/catkin_ws/src/src_loc/exp_results/ini_conds_{formatted_datetime}.csv"
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
    

    def run_exp(self):
        # initiate node
        rospy.init_node('run_exp_py', anonymous=True)

        algo_no = rospy.get_param("/algo_num")
        sensor_noise_no = rospy.get_param("/noise_flag")
        # only for gdts 
        grad_no = rospy.get_param("/grad_est")
        
        if(sensor_noise_no == 0):
           sensor_noise = "NoNoise" 
        else:
            sensor_noise = "Noise" 

        if(algo_no == 1):
            algo = "MTS"
        elif(algo_no == 2):
            algo = "GAMTS"
        elif(algo_no == 3):
            if(grad_no == 1):
                algo = "GDTS-LSM"
            elif(grad_no ==2):
                algo = "GDTS-FDM"
                

        # start the parameter initialzation launch file 
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_init_params = roslaunch.parent.ROSLaunchParent(uuid, ["/home/gnc001/catkin_ws/src/src_loc/launch/init_params.launch"])
        launch_init_params.start()
        rospy.loginfo("started ini_params")
        
        # start gazebo and sitl and mavros
        uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid1)
        launch_px4_sitl = roslaunch.parent.ROSLaunchParent(uuid1 , ["/home/gnc001/catkin_ws/src/src_loc/launch/px4_ros_minimal.launch"])
        launch_px4_sitl.start()
        rospy.sleep(rospy.Duration(10))
    
        mts_trej_node = roslaunch.core.Node(self.package, self.mts_trej_exec)
        sensor_node = roslaunch.core.Node(self.package, self.sensor_node_exec)
        mts_algo_node = roslaunch.core.Node(self.package, self.mts_algo_node_exec)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        
        process_sensor = launch.launch(sensor_node)

        current_datetime = datetime.datetime.now()
        formatted_datetime = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
        exp_data_fn = f"/home/gnc001/catkin_ws/src/src_loc/exp_results/{algo}_{sensor_noise}_exp_data_{formatted_datetime}.csv"


        df = pd.DataFrame(columns= ['var_idx i', 
                                    'src_loc_idx j',
                                    'sig_x',
                                    'sig_y',
                                    'psi_0',
                                    'pos_x_s',
                                    'pos_y_s',
                                    'loc_time']
                        )
        for i in range(self.vars_no):
            sig_x = self.vars[i][0]
            sig_y = self.vars[i][1]
            rospy.set_param('/sx', sig_x)
            rospy.set_param('/sy', sig_y)
            for j in range(self.src_loc_no):
                for psi_0 in range(0, 360, 30):
                    rospy.set_param('/yaw_ini', psi_0)
                    pos_x_s = self.source_locs[i][j][0]
                    pos_y_s = self.source_locs[i][j][1]
                    rospy.set_param('/pos_x_s', pos_x_s)
                    rospy.set_param('/pos_y_s', pos_y_s)
                    process_mts_trej = launch.launch(mts_trej_node)
                    process_mts_algo = launch.launch(mts_algo_node)
                    while (process_mts_trej.is_alive() and process_mts_algo.is_alive()):
                        pass    
                    process_mts_algo.stop()            
                    process_mts_trej.stop() 

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
                        df = df.append(data, ignore_index=True)
                        df.to_csv(exp_data_fn, index=False)
                    else :
                        rospy.logerr("something went wrong")

        launch_init_params.shutdown()
        launch.shutdown()
        process_sensor.stop()

if __name__ == "__main__":
    exp = random_trials()
    exp.gen_ini_conds()
    # exp.save_ini_conds()    
    exp.run_exp()