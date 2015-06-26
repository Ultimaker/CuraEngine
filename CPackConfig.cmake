set(CPACK_PACKAGE_VENDOR "Ultimaker")
set(CPACK_PACKAGE_CONTACT "Arjen Hiemstra <a.hiemstra@ultimaker.com>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Cura Engine")
set(CPACK_PACKAGE_VERSION_MAJOR 15)
set(CPACK_PACKAGE_VERSION_MINOR 05)
set(CPACK_PACKAGE_VERSION_PATCH 90)
set(CPACK_GENERATOR "DEB;RPM")

set(RPM_REQUIRES
    "arcus >= 15.05.90"
    "protobuf >= 3.0.0"
    "libstdc++6 >= 4.9.0"
    "libgcc1 >= 4.9.0"
)
string(REPLACE ";" "," RPM_REQUIRES "${RPM_REQUIRES}")
set(CPACK_RPM_PACKAGE_REQUIRES ${RPM_REQUIRES})

set(DEB_DEPENDS
    "arcus (>= 15.05.90)"
    "protobuf (>= 3.0.0)"
    "libstdc++6 (>= 4.9.0)"
    "libgcc1 (>= 4.9.0)"
)
string(REPLACE ";" ", " DEB_DEPENDS "${DEB_DEPENDS}")
set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEB_DEPENDS})

include(CPack)
