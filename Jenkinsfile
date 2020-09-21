pipeline {
  agent any
  stages {
    stage('Build and Test') {
      stages{
        stage('Build') {
          steps {
            sh './jenkins/build_centos7_gcc8.sh'
          }
        }
        stage('Test') {
          steps {
            sh './jenkins/test_centos7_gcc8.sh'
          }
        }
      }
    }
  }
}
