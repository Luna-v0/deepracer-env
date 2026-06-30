#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

class RewardDataPublisher(object):
    def __init__(self, agent_name, json_actions):
        print('Reward Distribution Graph: ', agent_name, json_actions)
        self.steering_angle_list = list()
        self.speed_list = list()
        self.action_space_len = len(json_actions)
        for json_action in json_actions:
            self.speed_list.append(str(json_action['speed']))
            self.steering_angle_list.append(str(json_action['steering_angle']))
        self.agent_name = agent_name
        # Note: the reward-distribution ROS topic publisher
        # (rospy.Publisher('/reward_data/<agent>', AgentRewardData)) has been removed.
        # The action-space data above is still populated; publish_frame is now a no-op.

    def publish_frame(self, image, action_index, reward_value):
        # No-op: reward-distribution publishing over the ROS master is not used here.
        # Kept for API compatibility (same signature/behavior contract for callers).
        pass

