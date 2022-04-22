import numpy as np
import time
from typing import Dict
import airsim
from airsim.types import MultirotorState


def stateToPosition(state: MultirotorState):
    return state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val

def positionToGrid(pos_x, pos_y):
    return (pos_x + 5) // 10, (pos_y + 5) // 10

def gridToPosition(grid_x, grid_y):
    return grid_x * 10, grid_y * 10

def nearEqu(pos, target):
    # not used now
    return abs(pos[0] - target[0]) < 0.05 and abs(pos[1] - target[1]) < 0.05

SPEED_OF_UAV = 3


# action 0 - 4
class EvaluationEnv:

    def __init__(self, uav_cnt):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

        self._state_dim = 10  # TODO: fake value
        self._uav_cnt = uav_cnt

        self._uav_grid_index = {}
        self._now_step = 0

        # TODO: generate obstacles @ wjf

    def _getObs(self):
        # TODO @ wjf
        return {
            f'uav_{i}': np.zeros((self._state_dim, )) for i in range(self._uav_cnt)
        }

    def _getRew(self):
        # TODO @ wjf
        return {
            f'uav_{i}': 0 for i in range(self._uav_cnt)
        }

    def _getDone(self):
        # TODO @ wjf
        done = {
            f'uav_{i}': self._now_step >= 500 for i in range(self._uav_cnt)
        }
        done['__all__'] = all(done.values())
        return done

    def _getInfo(self):
        # TODO @ wjf
        return {}
    
    def reset(self):
        self.client.reset()
        for i in range(self._uav_cnt):
            self.client.enableApiControl(True, vehicle_name=f"uav_{i}")
            self.client.armDisarm(True, vehicle_name=f"uav_{i}")

        # take off all UAVs
        p_future = []
        for i in range(self._uav_cnt):
            future = self.client.takeoffAsync(vehicle_name=f"uav_{i}")
            p_future.append(future)

        for future in p_future:
            future.join()

        self._uav_grid_index.clear()
        for i in range(self._uav_cnt):
            uav_name = f'uav_{i}'
            uav_pos = stateToPosition(self.client.getMultirotorState(vehicle_name=uav_name))
            self._uav_grid_index[uav_name] = positionToGrid(*uav_pos)

        self._now_step = 0
        return self._getObs()

    def step(self, actions):
        this_round_target = []
        dta = [(0, 0), (0, 1), (0, -1), (1, 0), (-1, 0)] # TODO: check action mapping
        
        p_future = []

        for i in range(self._uav_cnt):
            act = actions[f'uav_{i}']
            target_grid_pos = self._uav_grid_index[f'uav_{i}'][0] + dta[act][0], self._uav_grid_index[f'uav_{i}'][1] + dta[act][1]
            this_round_target.append(target_grid_pos)
            point = gridToPosition(*target_grid_pos)
            future = self.client.moveToPositionAsync(point[0], point[1], 0, SPEED_OF_UAV, vehicle_name=f'uav_{i}')
            p_future.append(future)

        for p in p_future:
            p.join()

        # all actions are done
        for i in range(self._uav_cnt):
            state = self.client.getMultirotorState(f'uav_{i}')
            pos = stateToPosition(state)
            grid_pos = positionToGrid(*pos)

            # print(f'Pos: {pos} MeasuredGridPos: {grid_pos} TargetGridPos: {this_round_target[i]}')
            # check    
            assert grid_pos == this_round_target[i]
            # update inner grid state
            self._uav_grid_index[f'uav_{i}'] = grid_pos
    
        self._now_step += 1

        return self._getObs(), self._getRew(), self._getDone(), self._getInfo()


def main():
    uav_number = 2
    env = EvaluationEnv(uav_number)
    state = env.reset()
    done = {'__all__': False}
    while not done['__all__']:
        actions = {f'uav_{i}': np.random.randint(0, 5) for i in range(uav_number)}
        print(actions)
        state, reward, done, info = env.step(actions)


if __name__ == '__main__':
    main()

