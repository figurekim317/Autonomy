Visual Studio 2017 x64 ������Ʈ�Դϴ�.

< ���� >
1. OpenCV ���̺귯�� ����
 - �ֽ� ���̺귯�� ���� ����� Ȳ���Դ� ������ �����Ͽ����ϴ�.
   https://www.youtube.com/watch?v=cRiDz0_5Wzc (opencv ����)
   https://www.youtube.com/watch?v=ptvnUCT7wEQ (opencv_contrib ���� ����)
 - cmake ���� ������ ������Ʈ�� VS 2017, 64bit �� �����մϴ�.
 - �� Image Stitching������ OpenCV SIFT�� ����ϰ� �ֽ��ϴ�.
   SIFT�� ����ϱ� ���ؼ��� cmake���� opencv_contrib �� �ݵ�� ���Խ��Ѿ� �ϰ�, (2��° ���� ����)
   cmake ���� �� enable_nonfree �ɼ��� �ݵ�� üũ�ؾ� �մϴ�.
 - �׸��� BUILD_opencv_world �ɼǵ� üũ�� �ݴϴ�.

2. OpenCV ���̺귯���� VS project���� ��ũ
 - ���� ���, OpenCV ���̺귯�� ���� ��θ� C:\opencv_build\my_opencv_build �� �Ͽ��ٸ�,
 - VS�� �޴����� ������Ʈ -> �Ӽ� -> C/C++ -> �Ϲ� -> �߰� ���� ���͸���
   C:\opencv_build\my_opencv_build\install\include �� �߰��� �ݴϴ�.
 - ������Ʈ -> �Ӽ� -> ��Ŀ -> �Ϲ� -> �߰� ���̺귯�� ���͸���
   C:\opencv_build\my_opencv_build\install\x64\vc15\lib �߰��� �ݴϴ�.
 - ������Ʈ -> �Ӽ� -> ��Ŀ -> �Է� -> �߰� ���Ӽ���
   opencv_world400d.lib �߰��� �ݴϴ�. (opencv ������ ���� �̸��� �޶��� �� ������, ������Ʈ�� Debug �� �� ������)

3. VS���� ����
 - ���� opencv_world400d.dll �� ���ٴ� ��Ÿ�� ������ �߻��ϸ� VS ������Ʈ ����� x64\Debug (�������� �ִ°�) ��
   C:\opencv_build\my_opencv_build\install\x64\vc15\bin\opencv_world400d.dll ������ ������ �ݴϴ�.

4. ����
 - ���� ������ stitching.exe �Դϴ�.
 - ���� ��� : stitching.exe [�̹���1] [�̹���2]
 - �̹���1, 2�� �̾� ���̰��� �ϴ� �� �̹��� ���� �̸��Դϴ�.
