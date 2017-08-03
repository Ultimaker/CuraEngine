parallel_nodes(["linux && cura", "windows && cura"]) {
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

                cmake '..', "-DCMAKE_PREFIX_PATH=\"${env.CURA_ENVIRONMENT_PATH}/${branch}\" -DCMAKE_BUILD_TYPE=Release"
                make
            }
        }
    }

    stage('Finalize') {
        notify_build_result(env.CURA_EMAIL_RECIPIENTS, '#cura-dev', ['master'])
    }
}
