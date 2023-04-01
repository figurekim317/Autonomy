Visual Studio 2017 x64 프로젝트입니다.

< 빌드 >
1. OpenCV 라이브러리 빌드
 - 최신 라이브러리 빌드 방법은 황선규님 영상을 참고하였습니다.
   https://www.youtube.com/watch?v=cRiDz0_5Wzc (opencv 빌드)
   https://www.youtube.com/watch?v=ptvnUCT7wEQ (opencv_contrib 포함 빌드)
 - cmake 에서 생성될 프로젝트는 VS 2017, 64bit 로 설정합니다.
 - 이 Image Stitching에서는 OpenCV SIFT를 사용하고 있습니다.
   SIFT를 사용하기 위해서는 cmake에서 opencv_contrib 를 반드시 포함시켜야 하고, (2번째 영상 참고)
   cmake 수행 시 enable_nonfree 옵션을 반드시 체크해야 합니다.
 - 그리고 BUILD_opencv_world 옵션도 체크해 줍니다.

2. OpenCV 라이브러리를 VS project에서 링크
 - 예를 들어, OpenCV 라이브러리 빌드 경로를 C:\opencv_build\my_opencv_build 로 하였다면,
 - VS의 메뉴에서 프로젝트 -> 속성 -> C/C++ -> 일반 -> 추가 포함 디렉터리에
   C:\opencv_build\my_opencv_build\install\include 를 추가해 줍니다.
 - 프로젝트 -> 속성 -> 링커 -> 일반 -> 추가 라이브러리 디렉터리에
   C:\opencv_build\my_opencv_build\install\x64\vc15\lib 추가해 줍니다.
 - 프로젝트 -> 속성 -> 링커 -> 입력 -> 추가 종속성에
   opencv_world400d.lib 추가해 줍니다. (opencv 버전에 따라 이름이 달라질 수 있으며, 프로젝트가 Debug 일 때 기준임)

3. VS에서 빌드
 - 만약 opencv_world400d.dll 이 없다는 런타임 에러가 발생하면 VS 프로젝트 경로의 x64\Debug (실행파일 있는곳) 에
   C:\opencv_build\my_opencv_build\install\x64\vc15\bin\opencv_world400d.dll 파일을 복사해 줍니다.

4. 실행
 - 실행 파일은 stitching.exe 입니다.
 - 실행 방법 : stitching.exe [이미지1] [이미지2]
 - 이미지1, 2는 이어 붙이고자 하는 두 이미지 파일 이름입니다.
