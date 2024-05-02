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


from pprint import pprint
import os
import sys
import yaml as pyyaml
from cortex.config import CRTXConfigPaths
from cortex.db import CORTEX_ENTITIES_DIR
from cortex.db.templates import SQLALCHEMY_TEMPLATE, SQLALCHEMY_IMPORT_TEMPLATE, FILE_HEADER, TEST_SCRIPT


class CRTXTableGenerator:
    # Class for generating SQLAlchemy code from CRTX table config
    def __init__(self, cortex_config_paths: CRTXConfigPaths = CRTXConfigPaths):
        # List of valid CRTX types
        self.__base_types = ["String", "Integer", "Double"]
        self.__array_types = [base_type + "[]" for base_type in self.__base_types]
        self.__column_types = self.__base_types + self.__array_types + ["DateTime"]
        self.__config_path = cortex_config_paths.database

        if not os.path.exists(self.__config_path):
            raise FileNotFoundError(f"Config path {self.__config_path} does not exist!")

        # Read the typing config file
        db_config = self.__read_config_file_as_yaml('typing')
        self.__type_map = {list(mapping.keys())[0]: list(mapping.values())[0] for mapping in db_config['cortex_type_map']}
        self.__qualifiers = db_config['cortex_qualifiers']

    def generate(self, output_dir: str = CORTEX_ENTITIES_DIR):
        # Generate SQLAlchemy code from CRTX table config
        hypertable_yaml = self.__read_config_file_as_yaml("hypertable")
        hypertable_code = []
        pprint(f"Hypertables:")
        pprint(hypertable_yaml)
        for config in hypertable_yaml:
            hypertable_code += self.__generate(config, "hypertable")
        CRTXTableGenerator.write(hypertable_code, 'hypertable', output_dir)

        reltable_yaml = self.__read_config_file_as_yaml("relational")
        reltable_code = []
        pprint(f"Relational tables:")
        pprint(reltable_yaml)
        for config in reltable_yaml:
            reltable_code += self.__generate(config, "relational")
        CRTXTableGenerator.write(reltable_code, 'relational', output_dir)

    @staticmethod
    def instantiate():
        from cortex.db import TemporalCRTX
        db = TemporalCRTX()
        db.shutdown(block=True)

    @staticmethod
    def write(code, table_type, output_dir):
        file = os.path.join(output_dir, f"{table_type}.py")

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        if os.path.exists(file):
            os.remove(file)
        open(file, 'a').close()

        with open(file, "w") as f:
            f.write(FILE_HEADER.replace("<table_type>", table_type))
            f.write(SQLALCHEMY_IMPORT_TEMPLATE)
            f.write("\n\n")
            f.write("\n".join(code))
            f.write(TEST_SCRIPT)

    def __get_config_file(self, name: str):
        # Return the config file for the specified table type
        if not os.path.exists(os.path.join(self.__config_path, f"{name}.yaml")):
            raise FileNotFoundError(
                f"Config path {self.__config_path} does not contain a {name}.yaml file!"
            )
        return os.path.join(self.__config_path, f"{name}.yaml")

    def __read_config_file_as_yaml(self, name: str):
        """Read the specified config file as YAML"""
        with open(self.__get_config_file(name), "r") as file:
            return pyyaml.load(file.read(), Loader=pyyaml.FullLoader)


    def __parse(self, table_yaml: str):
        # Convert YAML to python array of table config objects
        tables = pyyaml.load(table_yaml, Loader=pyyaml.FullLoader)
        return tables

    def __generate(self, table_config: dict, table_type: str = "hypertable"):
        # Convert python dict to SQLAlchemy code

        # Since the name config may contain multiple table names, we need to split them up
        # and create code for each table name (code will be the same for each table name)
        table_prefix = "ts" if table_type == "hypertable" else "rel"
        table_args = (
            """'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}"""
            if table_type == "hypertable"
            else ""
        )
        table_names = self.__get_table_names(table_config)

        columns = self.__get_columns(table_config, table_type)

        # Create SQLAlchemy code for each table name
        sqlalchemy_code = []
        for table_name in table_names:
            code = f"{SQLALCHEMY_TEMPLATE}"
            class_name_tokens = table_name.split("_")
            class_name = "".join([token.capitalize() for token in class_name_tokens])
            qualifiers = self.__get_table_qualifiers(table_config, table_name)

            if len(qualifiers) > 0:
                code = code.replace("INDICES", "\n\n" + "\n".join(qualifiers) + "\n\n")
            else:
                code = code.replace("INDICES", "\n")
            code = code.replace("CLASS_NAME", f"{class_name}")
            code = code.replace("TABLE_NAME", f"'{table_prefix}_{table_name}'")
            code = code.replace("TABLE_ARGS", table_args)
            code = code.replace("COLUMNS", "\n    ".join(columns))

            # Create printable representation using class name, column_name=self.column_value
            representation = f'f"""<{class_name}('
            for column in columns:
                column_name = column.split("=")[0].strip()
                representation += f"{column_name}='{{self.{column_name}}}', "
            representation = representation[:-2] + ')>"""'
            code = code.replace("REPRESENTATION", representation)

            sqlalchemy_code.append(code)

        return sqlalchemy_code

    def __get_columns(self, table_config: dict, table_type: str = "hypertable"):
        # Get columns from table config
        columns = []
        columns.append("id = Column(Integer, primary_key=True, autoincrement=True)")

        if table_type == "hypertable":
            # If hypertable, include the `time` and `msg_time` columns
            columns.append(
                "time = Column(DateTime(timezone=True), primary_key=True, nullable=False)"
            )
            columns.append("msg_time = Column(DateTime(timezone=True), nullable=True)")

        # Get value of the first element in the dict (since the dict only has one element)
        table_config = table_config[list(table_config.keys())[0]]

        for column_name, column_type in table_config.items():
            if "indices" not in column_name and "foreign_key" not in column_name:
                column_type, column_qualifiers = self.__parse_column(column_type)
                column_args = ""
                for qualifier in column_qualifiers:
                    column_args += f"{qualifier.lower()}=True, "

                # If nullable qualifier is not present, set it to false
                if "nullable" not in column_args:
                    column_args += "nullable=False"

                # Only add ', column_args' if column_args is not empty
                columns.append(
                    f"{column_name} = Column({column_type}{', ' + column_args if column_args else ''})"
                )
        return columns

    def __parse_column(self, column_type: str):
        tokens = column_type.split(",")

        # Validate column type
        if tokens[0] not in self.__column_types:
            raise ValueError(f"Invalid column type: {column_type}")

        column_type = self.__type_map[tokens[0]]
        column_qualifiers = tokens[1:] if len(tokens) > 1 else []

        # Remove whitespace from column qualifiers
        column_qualifiers = [qualifier.strip() for qualifier in column_qualifiers]

        # Validate column qualifiers
        for qualifier in column_qualifiers:
            if qualifier not in self.__qualifiers:
                raise ValueError(f"Invalid column qualifier: {qualifier}")

        return column_type, column_qualifiers

    def __get_table_qualifiers(self, table_config: dict, table_name: str):
        # Get table qualifiers from table config
        qualifiers = []
        table_config = table_config[list(table_config.keys())[0]]

        # Get column names to ensure the indices are valid
        column_names = [
            column_name
            for column_name in table_config.keys()
            if "indices" not in column_name and "foreign_key" not in column_name
        ]

        for column_name, column_type in table_config.items():
            if "indices" in column_name:
                indices = self.__parse_indices(column_type)

                for index in indices:
                    try:
                        self.__validate_index(index, column_names, table_name)
                    except ValueError as e:
                        # Print a warning to stderr and continue
                        print(f"WARNING: {e}", file=sys.stderr)
                        continue

                    idx_name = f"idx_{table_name}_{index.replace('CLASS_NAME.', '')}"
                    idx_name = idx_name.replace(", ", "_")
                    index_code = f"Index('{idx_name}', {''.join(index)})"
                    qualifiers += [index_code]
        return qualifiers

    def __validate_index(self, index: str, column_names: list, table_name):
        # Get column names from the indexes, which may be comma separated
        index_tokens = index.split(",")
        index_tokens = [token.strip() for token in index_tokens]

        # Get the part after the dot (e.g. CLASS_NAME.time -> time)
        index_tokens = [token.split(".")[-1] for token in index_tokens]

        # Check if all index tokens are valid column names
        for token in index_tokens:
            # If token is an empty string, yell at the user
            if token == "":
                # Print warning to stderr and continue
                print(
                    f"WARNING: Found empty string in {table_name} index configuration...",
                    file=sys.stderr,
                )
            if token not in column_names:
                raise ValueError(
                    f"Invalid index configuration, {table_name}.{token} does not exist! Skipping..."
                )

    def __parse_indices(self, qualifier: str):
        # Get indices from qualifier
        tokens = qualifier.split(",")
        indices = [index.strip() for index in tokens]
        index_args = []

        for index in indices:
            if "+" in index:
                index_tokens = index.split("+")
                index_tokens = [token.strip() for token in index_tokens]
                index_tokens = [f"CLASS_NAME.{token}" for token in index_tokens]
                index = ", ".join(index_tokens)
            else:
                index = f"CLASS_NAME.{index}"

            index_args.append(index)
        return index_args

    def __get_table_names(self, table_dict: dict):
        # Get table names from dict
        table_names = []
        for table_name in table_dict.keys():
            if "," in table_name:
                table_names += table_name.split(",")

                # Remove all characters not in [a-zA-Z_]
                table_names = [
                    "".join([char for char in name if char.isalpha() or char == "_"])
                    for name in table_names
                ]

                # Remove whitespace and convert to lowercase
                table_names = [name.strip().lower() for name in table_names]
            else:
                table_names.append(table_name)
        return sorted(list(set(table_names)))


def test():
    gen = CRTXTableGenerator()
    gen.generate()


if __name__ == "__main__":
    test()
