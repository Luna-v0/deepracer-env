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

'''This module implements concrete reset rule for crash'''

from deepracer_env.reset.abstract_reset_rule import AbstractResetRule
from deepracer_env.reset.constants import AgentCtrlStatus, AgentInfo
from deepracer_env.track_geom.track_data import TrackData
from deepracer_env.metrics.constants import EpisodeStatus

class CrashResetRule(AbstractResetRule):
    name = EpisodeStatus.CRASHED.value

    def __init__(self, agent_name, terminate_on_collision=True):
        '''Crash reset rule.

        Args:
            agent_name (str): racecar agent name.
            terminate_on_collision (bool): when True (default, faithful to AWS
                DeepRacer Object Avoidance), a detected collision flags the
                rule as done so the controller terminates / mercy-resets the
                episode. When False, the collision is still recorded
                (crashed_object_name is reported through agent_info, and the
                ``is_crashed`` reward param is set on the offending step), but
                ``done`` stays False so the cost signal stays alive across
                the full trajectory — used by D3's safety-1 cost level.
        '''
        super(CrashResetRule, self).__init__(CrashResetRule.name)
        self._track_data = TrackData.get_instance()
        self._agent_name = agent_name
        self._terminate_on_collision = terminate_on_collision

    def _update(self, agent_status):
        '''Update the crash reset rule done flag

        Args:
            agent_status (dict): agent status dictionary

        Returns:
            dict: dictionary contains the agent crash info
        '''
        crashed_object_name = self._track_data.get_collided_object_name(
            self._agent_name)
        is_collided = crashed_object_name != ''
        self._done = is_collided and self._terminate_on_collision
        return {AgentInfo.CRASHED_OBJECT_NAME.value: crashed_object_name,
                AgentInfo.START_NDIST.value: agent_status[AgentCtrlStatus.START_NDIST.value]}
