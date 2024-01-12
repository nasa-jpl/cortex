-- Use this script to print stuff at the end of initialization, useful for debugging

-- Get info about hypertables
-- SELECT * FROM timescaledb_information.hypertables;

-- Get info about chunk_time_interval
-- SELECT h.table_name, c.interval_length
--   FROM _timescaledb_catalog.dimension c
--   JOIN _timescaledb_catalog.hypertable h
--     ON h.id = c.hypertable_id;
