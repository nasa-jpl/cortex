-- This file should be placed in the /docker-entrypoint-initdb.d directory

-- Create default entries in some of the tables (if they don't already exist)

-- ----------------------------------------------------
--                       Robots                      --
-- ----------------------------------------------------
-- Insert Robots
-- INSERT INTO public.robots (id, name, description)
-- VALUES (1, 'Robot1', 'This is an example robot.')
-- ON CONFLICT DO NOTHING;

-- ----------------------------------------------------
--                       Actuators                   --
-- ----------------------------------------------------
-- Insert Actuators
-- INSERT INTO public.actuators (id, robot_id, name, type, make, model, serial_number)
-- VALUES (1, 1, 'joint_bend_01', 'bend', 'Acme', 'Snek', 'N/A')
-- ON CONFLICT DO NOTHING;

-- ----------------------------------------------------
--                   Known Devices                   --
-- ----------------------------------------------------
-- INSERT INTO public.devices (id, hostname, ip, arch, os, gpu, ram_bytes, disk_bytes, cpu_cores, cpu_freq_ghz)
-- VALUES (1, 'default', '192.168.10.100', 'x86_64', 'GNU/Linux', 0, 1024, 1024, 8, 2.4)
-- ON CONFLICT DO NOTHING;
