pipeline {
  agent any
  stages {
    stage('Build and Test') {
      parallel {
        stage('Local centos7 gcc8') {
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
        stage('Docker centos7 clang7') {
          stages{
            stage('Build') {
              steps {
                sh './jenkins/docker/build.sh centos7 clang7'
              }
            }
            stage('Test') {
              steps {
                sh './jenkins/docker/test.sh centos7 clang7'
              }
            }
          }
        }
        stage('Docker ubuntu20 gcc8') {
          stages{
            stage('Build') {
              steps {
                sh './jenkins/docker/build.sh ubuntu20 gcc8'
              }
            }
            stage('Test') {
              steps {
                sh './jenkins/docker/test.sh ubuntu20 gcc8'
              }
            }
          }
        }
        stage('Docker ubuntu20 clang7') {
          stages{
            stage('Build') {
              steps {
                sh './jenkins/docker/build.sh ubuntu20 clang7'
              }
            }
            stage('Test') {
              steps {
                sh './jenkins/docker/test.sh ubuntu20 clang7'
              }
            }
          }
        }
      }
    }
  }
}
