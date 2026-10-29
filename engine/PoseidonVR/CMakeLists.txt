# PoseidonVR - OpenXR integration for CWR-VR.
#
# Windows only for now (XR_KHR_opengl_enable on WGL); other platforms build a
# stub so the flat game is unaffected.  Makes no GL calls of its own — the
# graphics backend fills the swapchain images it hands out.

add_library(PoseidonVR STATIC
    VRSession.cpp
    VRSession.hpp
)

target_include_directories(PoseidonVR PUBLIC ${CMAKE_SOURCE_DIR}/engine)

if(WIN32)
    find_package(OpenXR CONFIG REQUIRED)
    target_link_libraries(PoseidonVR PRIVATE OpenXR::openxr_loader opengl32)
endif()

set_target_properties(PoseidonVR PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    OUTPUT_NAME "PoseidonVR"
)
