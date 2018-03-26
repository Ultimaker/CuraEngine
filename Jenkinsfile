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
                        extra_cmake_args = "-DArcus_DIR=\"${env.CURA_ENVIRONMENT_PATH}/${branch}/lib-mingw/cmake/Arcus\""
                    }

                    cmake '..', "-DCMAKE_PREFIX_PATH=\"${env.CURA_ENVIRONMENT_PATH}/${branch}\" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON ${extra_cmake_args}"
                    make('')
                }
                // Try and run the unit tests. If this stage fails, we consider the build to be "unstable".
                stage('Unit Test') {
                    try {
                        make('test')
                    } catch(e) {
                        currentBuild.result = "UNSTABLE"
                    }
                }
            }
        }

        stage('Finalize') {
            notify_build_result(env.CURA_EMAIL_RECIPIENTS, '#cura-dev', ['master'])
        }
    }
}
