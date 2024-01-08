-- This file should be placed in the /docker-entrypoint-initdb.d directory

-- JointStates table
create table if not exists ts_actual_joint_states
(
    time     timestamp with time zone not null,
    ros_time timestamp with time zone not null,
    robot    text                     not null,
    actuator text                     not null,
    position double precision         not null,
    velocity double precision         not null,
    effort   double precision         not null
);
alter table ts_actual_joint_states
    owner to postgres;
select create_hypertable('ts_actual_joint_states', 'time', chunk_time_interval := interval '1 day',
                         if_not_exists := TRUE);
create index if not exists ts_actual_joint_states_robot_actuator_idx ON ts_actual_joint_states (robot, actuator);


-- DesiredJointStates table
create table if not exists ts_desired_joint_states
(
    time     timestamp with time zone not null,
    ros_time timestamp with time zone not null,
    robot    text                     not null,
    actuator text                     not null,
    position double precision         not null,
    velocity double precision         not null,
    effort   double precision         not null
);
alter table ts_desired_joint_states
    owner to postgres;
select create_hypertable('ts_desired_joint_states', 'time', chunk_time_interval := interval '1 day',
                         if_not_exists := TRUE);
create index if not exists ts_desired_joint_states_robot_actuator_idx ON ts_desired_joint_states (robot, actuator);

-- Environments table
create table if not exists ts_environments
(
    time     timestamp with time zone not null,
    ros_time timestamp with time zone not null,
    host     text                     not null,
    name     text                     not null,
    value    text                     not null
);
alter table ts_environments
    owner to postgres;
select create_hypertable('ts_environments', 'time', chunk_time_interval := interval '1 day',
                         if_not_exists := TRUE);
create index if not exists ts_environments_host_idx ON ts_environments (host);
create index if not exists ts_environments_name_idx ON ts_environments (name);
create index if not exists ts_environments_value_idx ON ts_environments (value);


-- ForceTorqueSensors table
create table if not exists ts_force_torque_sensors
(
    time     timestamp with time zone not null,
    ros_time timestamp with time zone not null,
    robot    text                     not null,
    actuator text                     not null,
    force_x  double precision         not null,
    force_y  double precision         not null,
    force_z  double precision         not null,
    torque_x double precision         not null,
    torque_y double precision         not null,
    torque_z double precision         not null
);
alter table ts_force_torque_sensors
    owner to postgres;
select create_hypertable('ts_force_torque_sensors', 'time', chunk_time_interval := interval '1 day',
                         if_not_exists := TRUE);
create index if not exists ts_force_torque_sensors_robot_actuator_idx ON ts_force_torque_sensors (robot, actuator);


-- ResourceUtilization table
create table if not exists ts_resource_utilization
(
    time           timestamp with time zone not null,
    ros_time       timestamp with time zone not null,
    robot          text                     not null,
    node           text                     not null,
    host           text                     not null,
    proc_name      text                     not null,
    status         text                     not null,
    num_fds        integer                  not null,
    num_threads    integer                  not null,
    cpu_percent    double precision         not null,
    memory_percent double precision         not null
);
alter table ts_resource_utilization
    owner to postgres;
select create_hypertable('ts_resource_utilization', 'time', chunk_time_interval := interval '1 day',
                         if_not_exists := TRUE);
create index if not exists ts_resource_utilization_host_node_idx ON ts_resource_utilization (host, node);


-- Metrics table
create table if not exists ts_metrics
(
    time      timestamp with time zone not null,
    ros_time  timestamp with time zone not null,
    robot     text                     not null,
    name      text                     not null,
    value     double precision         null,
    value_str text                     null
);
alter table ts_metrics
    owner to postgres;
select create_hypertable('ts_metrics', 'time', chunk_time_interval := interval '1 day', if_not_exists := TRUE);
create index if not exists ts_metrics_robot_name_idx ON ts_metrics (robot, name);


-- Parameters table
create table if not exists ts_parameters
(
    time      timestamp with time zone not null,
    ros_time  timestamp with time zone not null,
    robot     text                     not null,
    name      text                     not null,
    value     double precision         null,
    value_str text                     null
);
alter table ts_parameters
    owner to postgres;
select create_hypertable('ts_parameters', 'time', chunk_time_interval := interval '1 day', if_not_exists := TRUE);
create index if not exists ts_parameters_robot_name_idx ON ts_parameters (robot, name);


-- Subsurface Controller States table
create table if not exists ts_state_transitions
(
    time           timestamp with time zone not null,
    ros_time       timestamp with time zone not null,
    robot          text                     not null,
    previous_state text                     not null,
    current_state  text                     not null,
    state_enum     smallint                 not null,
    state_str      text                     not null,
    module         text                     not null,
    message        text                     not null
);
alter table ts_state_transitions
    owner to postgres;
select create_hypertable('ts_state_transitions', 'time', chunk_time_interval := interval '1 day',
                         if_not_exists := TRUE);
create index if not exists ts_state_transitions_robot_idx ON ts_state_transitions (robot);


-- Health Monitor table
create table if not exists ts_health_monitors
(
    time     timestamp with time zone not null,
    ros_time timestamp with time zone not null,
    robot    text                     not null,
    level    text                     not null,
    name     text                     not null,
    label    text                     not null,
    message  text                     not null
);
alter table ts_health_monitors
    owner to postgres;
select create_hypertable('ts_health_monitors', 'time', chunk_time_interval := interval '1 day', if_not_exists := TRUE);
create index if not exists ts_health_monitors_robot_level_idx ON ts_health_monitors (robot, level);


-- Topic Statistics hypertable
create table if not exists ts_topic_statistics
(
    time             timestamp with time zone not null,
    ros_time         timestamp with time zone not null,
    robot            text                     not null,
    topic            text                     not null,
    node_sub         text                     not null,
    node_pub         text                     not null,
    start_time       timestamp with time zone not null,
    end_time         timestamp with time zone not null,
    dropped_msgs     integer                  not null,
    delivered_msgs   integer                  not null,
    traffic          integer                  not null,
    period_mean      INTERVAL                 not null,
    period_stddev    INTERVAL                 not null,
    period_max       INTERVAL                 not null,
    stamp_age_mean   INTERVAL                 not null,
    stamp_age_stddev INTERVAL                 not null,
    stamp_age_max    INTERVAL                 not null
);
alter table ts_topic_statistics
    owner to postgres;
select create_hypertable('ts_topic_statistics', 'time', chunk_time_interval := interval '1 day',
                         if_not_exists := TRUE);
create index if not exists ts_topic_statistics_robot_topic_idx ON ts_topic_statistics (robot, topic);


-- Odometry table
create table if not exists ts_odometry
(
    time        timestamp with time zone not null,
    ros_time    timestamp with time zone not null,
    robot       text                     not null,
    frame       text                     not null,
    position    geometry(PointZ)         not null,
    orientation geometry(PointZM)        not null,
    rpy         geometry(PointZ)         not null
);
alter table ts_odometry
    owner to postgres;
select create_hypertable('ts_odometry', 'time', chunk_time_interval := interval '1 day',
                         if_not_exists := TRUE);
create index if not exists ts_odometry_frame_id_idx ON ts_odometry (robot, frame);


create table if not exists ts_run_info
(
    start_time timestamp with time zone not null,
    end_time   timestamp with time zone not null,
    run_number integer                  null,
    run_name   text                     null,
    robot      text                     not null
);
alter table ts_run_info
    owner to postgres;
select create_hypertable('ts_run_info', 'start_time', chunk_time_interval := interval '1 day', if_not_exists := TRUE);
create index if not exists ts_run_info_end_time_idx ON ts_run_info (end_time);
create index if not exists ts_run_info_robot_idx ON ts_run_info (robot);


-- Annotations table
create table if not exists ts_annotations
(
    time     timestamp with time zone not null,
    time_end timestamp with time zone null,
    ros_time timestamp with time zone not null,
    host     text                     not null,
    text     text                     not null,
    tags     text[]                   not null
);
alter table ts_annotations
    owner to postgres;
select create_hypertable('ts_annotations', 'time', chunk_time_interval := interval '1 day', if_not_exists := TRUE);
create index if not exists ts_annotations_host_idx ON ts_annotations (host);
create index if not exists ts_annotations_name_idx ON ts_annotations (tags);