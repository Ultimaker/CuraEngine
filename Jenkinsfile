parallel_nodes(["linux && cura", "windows && cura"]) {
    timeout(time: 2, unit: "HOURS") {
        stage('Prepare') {
            step([$class: 'WsCleanup'])

            checkout scm
        }

        catchError {
            dir('build') {
                stage('Build') {
                    def branch = env.BRANCH_NAME
                    if(!fileExists("${env.CURA_ENVIRONMENT_PATH}/${branch}")) {
                        branch = "master"
                    }

                    extra_cmake_args = ""
                    if(!isUnix()) {
                        extra_cmake_args = "-DArcus_DIR=\"${env.CURA_ENVIRONMENT_PATH}/${branch}/lib-mingw/cmake/Arcus\" -DGTEST_LIBRARY=\"${env.CURA_ENVIRONMENT_PATH}/${branch}/lib-mingw/libgtest.a\" -DGTEST_MAIN_LIBRARY=\"${env.CURA_ENVIRONMENT_PATH}/${branch}/lib-mingw/libgtest_main.a\" -DGMOCK_LIBRARY=\"${env.CURA_ENVIRONMENT_PATH}/${branch}/lib-mingw/libgmock.a\" -DGMOCK_MAIN_LIBRARY=\"${env.CURA_ENVIRONMENT_PATH}/${branch}/lib-mingw/libgmock_main.a\""
                    }

                    cmake '..', "-DCMAKE_PREFIX_PATH=\"${env.CURA_ENVIRONMENT_PATH}/${branch}\" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON ${extra_cmake_args}"
                    make('')
                }
                // Try and run the unit tests. If this stage fails, we consider the build to be "unstable".
                stage('Unit Test') {
                    if (isUnix())
                    {
                        // For Linux
                        try {
                            sh 'make CTEST_OUTPUT_ON_FAILURE=TRUE test'
                        } catch(e)
                        {
                            currentBuild.result = "UNSTABLE"
                        }
                    }
                    else
                    {
                        // For Windows
                        try
                        {
                            // This also does code style checks.
                            bat 'ctest -V'
                        } catch(e)
                        {
                            currentBuild.result = "UNSTABLE"
                        }
                    }
                }
            }
        }

        stage('Finalize') {
            notify_build_result(env.CURA_EMAIL_RECIPIENTS, '#cura-dev', ['master'])
        }
    }
}
