-- Add a new database specifically for Grafana to store data in
SELECT 'CREATE DATABASE grafana'
WHERE NOT EXISTS (SELECT FROM pg_database WHERE datname = 'grafana');
\gexec
