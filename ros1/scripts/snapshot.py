#!/usr/bin/env python3

#  Copyright (c) 2024 Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#
#      https://www.apache.org/licenses/LICENSE-2.0
#
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import rospy
import rospkg
import subprocess
from datetime import datetime
from cortex.config import CRTXEnvironment
from cortex.db import TemporalCRTX
from cortex.db.entities import Environment, Workspace
from pathlib import Path
from time import sleep


def get_workspace_commit_hash(env, now_time, ros_time):
    # Get the ROS workspace path
    rospack = rospkg.RosPack()
    ros_paths = rospack.get_path('cortex')
    root_path = ros_paths.split('/cortex')[0]
    print(f"Looking for git commit hashes in {root_path}...")

    # For all repositories in the workspace, get the commit hash
    git_repos = list(Path(root_path).rglob('.git'))
    entities = []

    for repo in git_repos:
        repo_path = repo.parent

        # Get the commit hash
        try:
            git_proc = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=repo_path, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            commit_hash = git_proc.stdout.decode('utf-8').strip()

            git_proc = subprocess.run(['git', 'status', '--porcelain'], cwd=repo_path, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            uncommitted_changes = git_proc.stdout.decode('utf-8').strip()
            uncommitted_changes = len(uncommitted_changes.strip()) > 0

            git_proc = subprocess.run(['git', 'branch', '--show-current'], cwd=repo_path, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            branch_name = git_proc.stdout.decode('utf-8').strip()

            ws = Workspace(
                time=now_time,
                msg_time=ros_time,
                robot=env.system.ROBOT_NAME,
                host=env.device.HOSTNAME,
                package_name=f"{repo_path}".replace(f"{root_path}", ""),
                commit_hash=commit_hash,
                branch_name=branch_name,
                uncommitted_changes=uncommitted_changes,
            )

            entities.append(ws)
        except Exception as e:
            print(f"Failed to get commit hash for {repo_path}: {e}")

    return entities


def get_environment_variables(env, now_time, ros_time):
    envs = []
    for k, v in env:
        e = Environment(
            time=now_time,
            msg_time=ros_time,
            host=env.device.HOSTNAME,
            robot=env.system.ROBOT_NAME,
            key=k,
            value=v
        )
        envs.append(e)
    return envs


def main():
    # Setup DB connection
    env = CRTXEnvironment.local()
    db = TemporalCRTX(
        hostname=env.system.DB_HOSTNAME,
        port=env.system.DB_PORT,
        database=env.system.DB_NAME,
        batch_timeout=0.2
    )

    # Get time stamps
    now_time = datetime.now()
    try:
        import rospy
        ros_time = rospy.Time.now()
    except:
        ros_time = now_time

    try:
        env_vars = get_environment_variables(env, now_time, ros_time)
        db.insert(env_vars)

        hashes = get_workspace_commit_hash(env, now_time, ros_time)
        db.insert(hashes)
    except Exception as e:
        print(f"Failed to insert envs: {e}")

    sleep(5)
    db.shutdown(block=True)


if __name__ == '__main__':
    main()
