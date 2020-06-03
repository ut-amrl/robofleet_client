target_sources(client
PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/encoding.hpp
    ${CMAKE_CURRENT_LIST_DIR}/WsClient.hpp
    ${CMAKE_CURRENT_LIST_DIR}/RosClientNode.hpp
)
