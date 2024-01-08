-- This file should be placed in the /docker-entrypoint-initdb.d directory

-- Make sure TimescaleDB is installed
CREATE EXTENSION IF NOT EXISTS timescaledb CASCADE;

-- Add a schema specific to telegraf. The telegraf plugin uses a template that then creates
-- views in the public schema for consumption (JOIN's and such)
CREATE SCHEMA IF NOT EXISTS telegraf;

-- (Optional) Add the postgis extension
CREATE EXTENSION IF NOT EXISTS postgis CASCADE;
