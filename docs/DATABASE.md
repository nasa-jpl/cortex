# CORTEX Database Infrastructure

## PostgreSQL w/ TimescaleDB Plugin
### Relational + Time-Series
#### Benefits of TimescaleDB

## SQLAlchemy ORM
### Pythonic Database Interactions

## `TemporalCRTX` Class
### Basic Usage
#### Instantiation
#### Inserting
[//]: # (TODO: show how SQLAlchemy is used )
#### Querying Data
### Batching
#### Max Batch Size
#### Batch Timeout

## Configuration
### Sensible Defaults
### Customization
CORTEX provides an easy method for customizing the underlying database schema using
a declarative YAML configuration file. The configuration file is used to define the
database schema, including tables, columns, and relationships. The configuration file
is then used to generate SQLAlchemy models, which can be used to interact with the
database.
#### Creating a Custom Schema
#### Generating SQLAlchemy Models


