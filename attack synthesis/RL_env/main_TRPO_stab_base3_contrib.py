import logging
from tabnanny import verbose
import gym
import torch
from sb3_contrib import TRPO
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
from stable_baselines3.common.env_util import make_vec_env
import simple_driving
import time

##PARAMETERS
PHYSICAL_FRAMES = 100
REWARD_THRESHOLD = 200
TOTAL_TIMESTEPS = 1000_000
LOG_INTERVAL = 1
GOAL_MOVE = 100 #2*goal_move should not be greater than 200 (dist to goal to stop simulaiton)
saved_model_name = 'trpo_MLP_PHYSICAL_FRAMES_'+str(PHYSICAL_FRAMES)+'_REWARD_THRESHOLD_'+str(REWARD_THRESHOLD)+'_TOTAL_TIMESTEPS_'+str(TOTAL_TIMESTEPS)+'_move_goal_'+str(GOAL_MOVE)+'_detect_motors_off_penalize_40'
testing = True
rendering = True
def main():
    env = gym.make('SimpleAttack-v0', render = rendering, physical_frames = PHYSICAL_FRAMES, goal_move=GOAL_MOVE, logging=testing)

    if not(testing):
        try:
            # #check_env(env=env)
            model = TRPO("MlpPolicy", env, verbose=2, tensorboard_log="./trpo_log_with_new_features/") #MlpPolicy
            callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=REWARD_THRESHOLD, verbose=1)
            eval_callback = EvalCallback(env, callback_on_new_best=callback_on_best, verbose=1, best_model_save_path='./best_model_callback/')
            model.learn(total_timesteps=TOTAL_TIMESTEPS, log_interval = LOG_INTERVAL, callback=eval_callback)
            model.save(saved_model_name)
        except KeyboardInterrupt:
            model.save("trpo_log_with_new_features_killed_process")
        
    if testing:
        model = TRPO.load(saved_model_name) #'./best_model_callback/best_model') # 

        obs = env.reset()
        sum_rewards = 0
        while True:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            sum_rewards = sum_rewards + reward
            #env.render()
            if done:
                # print(sum_rewards)
                obs = env.reset()
                sum_rewards = 0


if __name__ == '__main__':
    main()
