## Run unit test

To run unit tests locally on the entire MoveIt catkin workspace using catkin-tools:
```shell
 $ catkin run_tests -iv
```


To run a test for just 1 package:
```shell
 $ catkin run_tests --no-deps --this -iv
```


To ignore most of the log/print output of the tests:
```shell
 $ catkin run_tests --no-status --summarize --no-deps --this
```
