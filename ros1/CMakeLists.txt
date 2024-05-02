cmake_minimum_required(VERSION 3.16)
project(cortex)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
    Annotation.msg
    AnnotationTags.msg
    AnnotationLevel.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

# add_custom_target(InstallDocker ALL
#     COMMAND rm -rf dist && unzip urdf-viewer-prod.zip
#     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/
# )

# add_custom_target(DockerUpdate ALL
#     COMMAND docker compose down && docker compose up -d
#     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/
# )
