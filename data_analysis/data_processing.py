import pandas as pd 

# File names
csv_dir = 'logs/'
csv_file1 = 'MultipleFrequenciesAttack_'
csv_file2 = 'IMU_at_true_speed_'            # (BMI270) Frequency attack on our board with logging at sensor speed.
csv_file3 = 'Pixhawk_multiple_freq_'
csv_file4 = 'PixHawk6C_RT_multipleFreq_'    # (BMI055) Frequency attack on PixHawk6C board with logging at sensor speed.
csv_file5 = 'suspend_bmi055_v7_'            # (BMI055) Suspend attack
csv_file6 = 'Invensensev3Fequency_'         # (ICM42688)
csv_file7 = 'InvensenseV3Suspend_'          # (ICM42688)
csv_file2 = 'suspend_with_true_speed_IMU_'
csv_file2 = 'suspend_'

# kakute_board = True
frequency_attack = False
IMU_list = ['BMI270', 'BMI055', 'ICM42688', 'MPU6000', 'PIXHAWK6c']
experiment_data = {'BMI270': 
                    {'suspend_file': 'suspend_',
                     'freq_file': 'frequency_all_possible_',
                     'freq_map':{0:3200, 1:1600, 2:800, 3:400, 4:200, 5:100, 6:50, 7:25, 8:25/2, 9:25/4, 10:25/8, 11:25/16 ,12:25/32}},
                    'BMI055':
                    {'suspend_file': csv_file5,
                     'freq_file': 'frequency_all_possible_',
                     'freq_map':{15:1000,14:500,13:250,12:125,11:125/2,10:125/4,9:125/8,8:125/16}},
                    'MPU6000':
                    {'suspend_file': 'InvensenseSuspend_',
                     'freq_file': 'InvensenseFrequency_',
                     'freq_map':{0:256, 1:188, 2:98, 3:42, 4:20, 5:10, 6:5}},
                    'ICM42688':
                    {'suspend_file': 'Invensensev3Suspend_',
                     'freq_file': 'Invensensev3Fequency_',
                     'freq_map':{0:2000, 1:1000, 2:200, 3:100, 4:50, 5:25, 6:12.5}}, # unsigned char frequencies_gyro[7] =  {0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B};
                    'PIXHAWK6c':
                    {'suspend_file': 'suspend_both_',
                     'freq_file': 'frequency_both_',
                     'freq_map':{0:2000, 1:1000, 2:200, 3:100, 4:50, 5:25, 6:12.5}}, # unsigned char frequencies_gyro[7] =  {0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B};
                    }


IMU = IMU_list[3]
csv_dir += str(IMU) + '/'

freq_map_file2 = {0:1600, 1:800, 2:400, 3:200, 4:100, 5:50, 6:25}
freq_map_file4 = {15:1000, 14:500, 13:250, 12:125, 11:62.5, 10:31.25, 9:15.63}
freq_map_MPU6000 = {0:256, 1:188, 2:98, 3:42, 4:20, 5:10, 6:5}

def load_suspend_data_pixhawk_invensense(datasets, imu):
    datasets += ['GT2']
    csv_file = 'Suspend/'+ experiment_data[imu]['suspend_file']
    df_dict = load_csv_files(datasets, imu, csv_file)
    return df_dict

def load_suspend_data(datasets, imu):
    datasets += ['GT']
    csv_file = 'Suspend/'+ experiment_data[imu]['suspend_file']
    df_dict = load_csv_files(datasets, imu, csv_file)
    return df_dict

def load_frequency_data(datasets, imu):
    datasets += ['AC']
    csv_file = 'Frequency/'+experiment_data[imu]['freq_file']
    df_dict = load_csv_files(datasets, imu, csv_file)

    df = df_dict['AC']
    freq_map = experiment_data[imu]['freq_map']
    # load frequency map for frequency attacks
    df2 = df.replace({"Ground_Truth":freq_map})
    df['Ground_Truth'] = df2['Ground_Truth']
    df_dict['AC'] = df
    return df_dict

def load_frequency_data_pixhawk_invensense(datasets, imu):
    datasets += ['AC2']
    csv_file = 'Frequency/'+experiment_data[imu]['freq_file']
    df_dict = load_csv_files(datasets, imu, csv_file)

    df = df_dict['AC2']
    freq_map = experiment_data[imu]['freq_map']
    # load frequency map for frequency attacks
    df2 = df.replace({"Ground_Truth":freq_map})
    df['Ground_Truth'] = df2['Ground_Truth']
    df_dict['AC'] = df
    return df_dict

def load_csv_files(datasets, imu, csv_file):
    df_dict = dict()
    csv_dir = 'logs/'+imu+'/'
    
    for ds in datasets:
        #print('loading: ',csv_dir+csv_file+ds+'.csv')
        df = pd.read_csv(csv_dir+csv_file+ds+'.csv', on_bad_lines='skip')
        df_dict[ds]= df
    return df_dict
    

def compute_transition_df(source_df, param):
    # dataframe for attack transition
    df_transition = pd.DataFrame()
    df_transition['timestamp'] = source_df['timestamp']
    df_transition[param] = source_df[param]
    df_transition['transition'] = source_df[param].diff()
    df_transition = df_transition.loc[df_transition['transition']!=0].dropna()
    return df_transition