pipeline {
  agent any
  stages {
    stage('Build and Test') {
      parallel {
        stage('Local') {
          stages{
            stage('Build local') {
              steps {
                sh './jenkins/build.sh'
              }
            }
            stage('Test local') {
              steps {
                sh './jenkins/test.sh'
              }
            }
          }
        }
        stage('Docker centos') {
          stages{
            stage('Build docker centos') {
              steps {
                sh './jenkins/docker/build.sh centos gcc'
              }
            }
            stage('Test docker centos') {
              steps {
                sh './jenkins/docker/test.sh centos gcc'
              }
            }
          }
        }
      }
    }
  }
}
