set(hdrs Mapper.h
    PARENT_SCOPE
    MapServiceSVC_impl.h
    MapBuilder.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

