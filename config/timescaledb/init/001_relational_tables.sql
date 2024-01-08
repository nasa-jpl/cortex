-- This file should be placed in the /docker-entrypoint-initdb.d directory

-- Robots table
create table if not exists rel_robots
(
    id             serial,
    name           varchar not null,
    description    varchar,
    primary key (id),
    unique (name)
);
alter table rel_robots
    owner to postgres;
CREATE INDEX IF NOT EXISTS robots_model_name_idx ON rel_robots(name);


-- Actuators table
create table if not exists rel_actuators
(
    id            serial,
    robot_id      integer not null,
    name          varchar not null,
    type          varchar,
    make          varchar,
    model         varchar,
    serial_number varchar,
    primary key (id),
    foreign key (robot_id) references rel_robots
);
alter table rel_actuators
    owner to postgres;
CREATE INDEX IF NOT EXISTS actuators_name_idx ON rel_actuators(name);


-- Devices table
create table if not exists rel_devices
(
    id         serial,
    hostname   varchar not null,
    ip         varchar not null,
    arch       varchar not null,
    os         varchar not null,
    gpu        integer not null,
    ram_bytes  bigint  not null,
    disk_bytes bigint  not null,
    cpu_cores  integer not null,
    cpu_freq_ghz   double precision not null,
    primary key (id)
);
alter table rel_devices
    owner to postgres;


-- ROS Nodes table
create table if not exists rel_ros_nodes
(
    id            serial,
    name          varchar not null,
    primary key (id),
    unique (name)
);
alter table rel_ros_nodes
    owner to postgres;


-- Workspace commits table
create table if not exists rel_repository_commits
(
    id     serial,
    module varchar   not null,
    branch varchar   not null,
    commit timestamp not null,
    primary key (id)
);
alter table rel_repository_commits
    owner to postgres;
