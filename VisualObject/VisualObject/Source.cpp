#pragma hdrstop
#pragma argsused
#include <tchar.h>
#include <stdio.h>
#include  <GL/glut.h>
#include <Windows.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>


using namespace std;



// ������� �������� ������**********************************************/

//���������� ��� ������ ����� �� �����
string line;
//���������� ������ �� �������� ���� ����
ifstream My_File_obj;
//����� ������� ����������� ���������� �����
int  coords_index = 0, Fase_index = 0;


struct coord
{
	float x;
	float y;
	float z;
};

struct polygons
{
	coord* point; //1 �����
};
//�����
coord* MyMass;
//��������
polygons* models;
//���������� �����
int point = 1;
//�������� ���������� � �������
int p = 0;


//�������� ������ � ������ ���������
void Compare(float* Coords, int* indexs, int count_fase, int count_point)
{
	//������ ������ ��� �����
	MyMass = new  coord[count_point / 3];
	//������ ������ ��� ���������
	models = new polygons[count_fase / 4];

	//���������� ����� � ����������� ���������� ������ � ������ 1
	for (int i = 0; i < count_point; i += 3)
	{
		MyMass[point].x = Coords[i + 0]; //���������� �
		MyMass[point].y = Coords[i + 1]; //���������� Y
		MyMass[point].z = Coords[i + 2]; //���������� Z
		point++;
	}
	p = 0;
	//���������� ����� � ����������� ���������� � ����������� �� ������� �����
	for (int f = 0; f < count_fase; f += 3)
	{
		models[p].point = new coord[3];
		for (int s = 0; s < 3; s++)
			models[p].point[s] = MyMass[indexs[f + s]];

		p++;
	}

}

//������ ����
void Rad_file_obj(string name_file)
{
	My_File_obj.open(name_file);
	//��������� ����
	if (My_File_obj.is_open())
	{ //���� ������� ����� ������ ������ �� �����
		cout << "file open";
		//������������ ���� �� �����
		My_File_obj.seekg(0, ios::end);
		//�������� ���������� ��������� �� ����� ��������
		int  File_Size = My_File_obj.tellg();
		//������ ������ ���������� ������� ��� ���������
		float* Tmp_Coords = (float*)malloc(File_Size);
		//������ ������ ���������� ������� �������� ����� �����������
		int* Tmp_faseArray = (int*)malloc(File_Size);
		//������������� � 0 ������� �������
		coords_index = 0;
		Fase_index = 0;
		//������������� ������ � ������ �����
		My_File_obj.seekg(0, ios::beg);

		int countpoint = 0;
		int countfase = 0;
		//������ ���� �� �����
		while (!My_File_obj.eof())
		{
			//������ ������ ������ �� �����
			getline(My_File_obj, line);
			replace(line.begin(), line.end(), '.', ',');
			//��������� ������ ��� ������� ������
			if ((line.c_str()[0] == 'v') && (line.c_str()[1] == ' '))
			{
				//�������� 1 ������
				line[0] = ' ';
				//�������� 1 ������
				line[1] = ' ';
				//����� ������� ��������� ��������� ���������� �������� ������ ������
				sscanf_s(line.c_str(), "%f %f %f ",
					&Tmp_Coords[coords_index + 0],   //���������� X
					&Tmp_Coords[coords_index + 1], //���������� Y
					&Tmp_Coords[coords_index + 2]  //���������� Z
				);



				//�������� ������ �� 3 ��� ��� 3 ����������
				coords_index += 3;
				//������ �����
				countpoint++;
			}
			//��������� �� ����� �� �����
			if ((line.c_str()[0] == 'f') && (line.c_str()[1] == ' '))
			{
				//�������� 1 ������
				line[0] = ' ';
				//�������� ��������� �����������
				//     v1       v2     v3      v3
				int tmp_point[4], tmp_normal[4], tmp_texture[4];
				sscanf_s(line.c_str(), "%i/%i/%i%i/%i/%i%i/%i/%i",
					// ����� �����  ����� �������   ����� ��������
					&tmp_point[0], &tmp_normal[0], &tmp_texture[0]  //p1
					, &tmp_point[1], &tmp_normal[1], &tmp_texture[1]  //p2
					, &tmp_point[2], &tmp_normal[2], &tmp_texture[2]  //p3
				//	, &tmp_point[3], &tmp_normal[3], &tmp_texture[3]  //p4
				);

				//��������� ������� � ������
				Tmp_faseArray[Fase_index + 0] = tmp_point[0]; //��������� ������ ������ �����
				Tmp_faseArray[Fase_index + 1] = tmp_point[1]; //��������� ������ ������ �����
				Tmp_faseArray[Fase_index + 2] = tmp_point[2]; //��������� ������ ������ �����
				//Tmp_faseArray[Fase_index + 3] = tmp_point[3]; //��������� ��������� ������ �����



				Fase_index += 3;
				countfase++;
			}
		}

		//���������� ��������� ��������� ��� ���������
		Compare(Tmp_Coords, Tmp_faseArray, Fase_index, coords_index);
	}
	else
	{
		//���� �� ������� �� ����� ������
		cout << "file not open";
	}


}

GLdouble x, y, z;
POINT mouseposition;


GLfloat  tx = 0;            // ����� �� ��� X
GLfloat     ty = 0;            // Y
GLfloat     tz = 0;            // Z
GLfloat  rx = 0;            // ���� ������� ����� ������ ��� X
GLfloat  ry = 0;            // Y
GLint     tt = 0;            // �������� ��������: 0 - XY, 1 - XZ

int mx = 0, my = 0;                // ���������� ����
bool ldown = false,        // ������ ����� ������� ����?
rdown = false;
int point_number = 0;
coord six_point[5] = { 0,0,0,0,0 };

float size1 = 0.1;

//void getIntersect();

void Draw_Obj()
{
	glEnable(GL_ALPHA_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_LIGHTING);
	glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glEnable(GL_NORMALIZE);
	float light0_diffuse[] = { 1, 1, 1 };
	float light0_direction[] = { -1, -1, 0, 100 };
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, light0_direction);


	for (int i = 0; i < p; i++)
	{
		//������ ������ �� �����
		//GL_POLYGON  GL_POINTS   GL_LINES GL_POLYGON GL_QUADS
		glColor3f(0, 0, 0);
		glBegin(GL_LINE_LOOP);
		for (int s = 0; s < 3; s++)
			glVertex3f(models[i].point[s].x, models[i].point[s].y, models[i].point[s].z);
		glEnd();
		//������ ��������
		glColor3f(1, 1, 1);
		glBegin(GL_POLYGON);
		for (int s = 0; s < 3; s++)
			glVertex3f(models[i].point[s].x, models[i].point[s].y, models[i].point[s].z);
		glEnd();
	}
	/*
	for (int i = 0; i < 6; ++i)
	{
		if ((six_point[i].x != 0) & (six_point[i].y != 0) &(six_point[i].z != 0))
		{
			glPointSize(10);
			glBegin(GL_POINTS);
			glColor3f(0, 1, 0);
			glVertex3f(six_point[i].x, six_point[i].y, six_point[i].z);
			glEnd();
		}
	}
	*/


	//getIntersect();

}


//������������ �������
/*
void getIntersect() {
	GetCursorPos(&mouseposition);
	GLint viewport[4];       // ��������� viewport-a.
	GLdouble projection[16]; // ������� ��������.
	GLdouble modelview[16];  // ������� �������.
	GLsizei vx, vy;          // ���������� ������� ���� � ������� ��������� viewport-a.
	GLint xx, yy = 0;		// ��� �������� �������
	glGetIntegerv(GL_VIEWPORT, viewport);           // ����� ��������� viewport-a.
	xx = glutGet(GLUT_WINDOW_X);
	yy = glutGet(GLUT_WINDOW_Y);
	glGetDoublev(GL_PROJECTION_MATRIX, projection); // ����� ������� ��������.
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);   // ����� ������� �������.
	vx = mouseposition.x - xx;
	vy = viewport[3] - mouseposition.y + yy;
	gluUnProject(vx, vy, 0.5, modelview, projection, viewport, &x, &y, &z);
	//cout << "x = " << x << " y = " << y << " z = " << z << endl;
	glPointSize(10);
	glBegin(GL_POINTS);
	glColor3f(1, 0, 0);
	glVertex3f(x, y, z);
	glEnd();
}
*/


void MouseMotion(int x, int y)    //����������� ����
{
	if (ldown)        // ����� ������
	{
		rx += 0.5 * (y - my);    //��������� ����� ��������
		ry += 0.5 * (x - mx);
		mx = x;
		my = y;
		//    glutPostRedisplay();    //������������ �����
	}

	if (rdown)    //������
	{
		tx += 0.01 * (x - mx);    //����������� ����� �������� ���������
		if (tt)
			tz += 0.01 * (y - my);
		else
			ty += 0.01 * (my - y);
		mx = x;
		my = y;
	}
	cout << "\n X=" << tx << " Y= " << ty << " Z=" << tz;
}


void Mouse(int button, int state, int x, int y)        //��������� ������� ����
{
	cout << "\nbutton " << button << "state" << state;

	if (button == GLUT_LEFT_BUTTON)        //����� ������
	{
		switch (state)
		{
		case GLUT_DOWN:        //���� ������
			ldown = true;        //���������� ����
			mx = x;            //��������� ����������
			my = y;
			break;
		case GLUT_UP:
			ldown = false;
			break;
		}
	}
	if (button == GLUT_RIGHT_BUTTON)    //������ ������
	{
		switch (state)
		{
		case GLUT_DOWN:
			rdown = true;
			mx = x;
			my = y;
			break;
		case GLUT_UP:
			rdown = false;
			break;
		}
	}
}


void Display()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);

	glColor3f(0.1, 0.7, 0.2);
	glClearColor(0.5, 0.5, 0.75, 1);

	//    glClearColor(0.5f, 0.5f, 0.5f, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	glPushMatrix();
	glScalef(size1, size1, size1);        //����������� � ������ �������
	glScaled(size1, size1, size1);
	glTranslatef(tx, ty, tz);
	glRotatef(rx, 1, 0, 0);
	glRotatef(ry, 0, 1, 0);
	Draw_Obj();            //����� ������� �� �����

	//getIntersect();

	glPopMatrix();
	glutSwapBuffers();
}

void Keyboard(unsigned char key, int xx, int yy)            //��������� ��������� �� ����������
{
	switch (key)
	{
	case VK_ESCAPE:        //���� ������ ������� ESC - �����
		//exit(0);
		break;
	case 'q':        //���� ������ ������� ESC - �����
		size1 += 0.001;
		break;
	case 'w':        //���� ������ ������� ESC - �����
		size1 -= 0.001;
		break;

		// ����������� ��������� 
		/*
		case 'e':
			if (point_number == 5)
				point_number = 0;
			six_point[point_number].x = x;
			six_point[point_number].y = y;
			six_point[point_number].z = z;
			cout << endl << point_number + 1 << " : " << six_point[point_number].x << ' ' << six_point[point_number].y << ' ' << six_point[point_number].z;
			point_number++;
		*/
	}

	if (size1 < 0) size1 = 0;

}

void IDLE()
{
	glutPostRedisplay();
}

void InitOpenGL(int argc, _TCHAR* argv[])
{


	//�������� ��������� OpenGL � ����������
	glutInit(&argc, reinterpret_cast<char**> (argv));
	//������ ��������� ��������� ��������
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	//������������� ������� ���� ��� ��������
	glutInitWindowSize(640, 480);
	//������������� ������� ����
	glutInitWindowPosition(100, 100);
	//������� ���� � ��������� ��� ���������
	glutCreateWindow("My first Pack Man");
	//�������� ������� ��������� ����
	glutDisplayFunc(Display);
	//��������� ������� ������
	glutKeyboardFunc(Keyboard);

	glutMouseFunc(Mouse);

	glutMotionFunc(MouseMotion);

	glutIdleFunc(IDLE);

	//��������� ����������� ���� ������ ����������
	glutMainLoop();
}


/**************************************************************************/


int main(int argc, _TCHAR* argv[])
{
	setlocale(LC_ALL, "Russian");
	//��������� ����
	cout << "Loading cube2.obj";
	Rad_file_obj("CUBE.obj");

	//��������� ����������� �������� � ������ �����������
	My_File_obj.close();
	//��������� OpenGL
	InitOpenGL(argc, argv);
	return 0;
}