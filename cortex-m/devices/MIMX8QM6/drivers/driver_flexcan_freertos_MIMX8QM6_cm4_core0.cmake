if(NOT DRIVER_FLEXCAN_FREERTOS_MIMX8QM6_cm4_core0_INCLUDED)
    
    set(DRIVER_FLEXCAN_FREERTOS_MIMX8QM6_cm4_core0_INCLUDED true CACHE BOOL "driver_flexcan_freertos component is included.")

    target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/fsl_flexcan_freertos.c
    )


    target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/.
    )


    include(driver_flexcan_MIMX8QM6_cm4_core0)

    include(middleware_freertos-kernel_MIMX8QM6_cm4_core0)

endif()
