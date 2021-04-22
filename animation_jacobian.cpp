#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <shader.h>
#include <sphere1.h>
#include <spline.h>
#include <linkjoint.h>



#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>


void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mode);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_pos_callback(GLFWwindow* window, double xpos, double ypos);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);


void drawnode(linkjoint* root, Shader spline, int elindices);
void findinverse(float** a);
void multiplymat(float** a, float** b, int n1, int n2, int n3, float** c);
void transposemat(float** a, int n1, int n2, float** a2);
void pinverse(float** a, int n, float** b);
float determinant(float** a, int n);
void movenode(linkjoint* node);

using namespace std;
float swt = 1;
float fov = 45.0f;

bool movemode = true;
bool movestart = false;
int jointnum = 0;


glm::vec3 cameraPos = glm::vec3(0.0f, 100.0f, 220.0f);
glm::vec3 dronefront = glm::vec3(1.0f, 0.0f, -1.0f);

glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
glm::vec3 centerofrot(0.0f, 0.0f, 0.0f);



glm::vec3 posvec;
glm::vec3 rotate_axis;
float rotate_theta;

vec3 goalpos(20, -16, 0);

float oritheta = 0;
vec3 oriaxis(1, 0, 0);
quat goalori = angleAxis(oritheta, oriaxis);

vector<linkjoint*> allnode;




int main() {
	//-------------------------------여기부터-------------------------------------
	glfwInit(); //glfw초기화
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);//(우리가 설정할 옵션, 옵션값)
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(800, 600, "LearnOpenGL", NULL, NULL); //(높이, 너비,창이름, null, null) 객체생성

	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window); //window를 주context로 지정
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback); //창의 크기가 변경될때마다 함수를 호출하도록 등록

	//GLAD
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))//glad를 초기화
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}
	//--------------------------여기까지 no touch (기본설정)--------------------------
	float square[] = {
-1.0f, 0.0f, -1.0f, 0.0f,  1.0f, 0.0f,
 1.0f, 0.0f, -1.0f, 0.0f,  1.0f, 0.0f,
 1.0f, 0.0f,  1.0f, 0.0f,  1.0f, 0.0f,
 1.0f, 0.0f,  1.0f, 0.0f,  1.0f, 0.0f,
-1.0f, 0.0f,  1.0f, 0.0f,  1.0f, 0.0f,
-1.0f, 0.0f, -1.0f, 0.0f,  1.0f, 0.0f,
	};

	//square--------------------------------------
	unsigned int squarevao, squarevbo;
	glGenBuffers(1, &squarevbo); //버퍼 id를 생성
	glGenVertexArrays(1, &squarevao);

	glBindVertexArray(squarevao); //array 바인딩
	glBindBuffer(GL_ARRAY_BUFFER, squarevbo);//버퍼의 유형을 선택하여 바인딩
	glBufferData(GL_ARRAY_BUFFER, sizeof(square), square, GL_STATIC_DRAW);//데이터를 버퍼의 메모리에 저장

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);	//vertex 좌표
	//texture attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);	//normal vector 좌표


	//sphere------------------------------------------------------
	value vec;
	vec = sphere(1, 40);
	vector<float> ver = vec.vertices;
	vector<int> ind = vec.indices;
	vector<int> ind_grid = vec.indices_grid;

	unsigned int sphereVbo;
	unsigned int sphereEbo;
	unsigned int sphereVao;
	//색칠-----------------------
	glGenBuffers(1, &sphereVbo);
	glGenBuffers(1, &sphereEbo);
	glGenVertexArrays(1, &sphereVao);

	glBindVertexArray(sphereVao);
	glBindBuffer(GL_ARRAY_BUFFER, sphereVbo);
	glBufferData(GL_ARRAY_BUFFER, ver.size() * sizeof(float), &ver[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphereEbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, ind.size() * sizeof(float), &ind[0], GL_STATIC_DRAW);


	//propeller-----------------------------------------------------------------------------------------------------------------------
	int ct = 4; //단면을 구성하는 control point 개수
	int step1 = 30; //단면쪼개는횟수
	int step2 = 5; //swept쪼개는 횟수

	int clockwise = 1;//단면의 좌표들이 배치된 방향 (시계방향이면 -1)


	sweptsurface element("element.txt", ct, step2, step1, clockwise);
	vector<float> elcoord = element.sweptcoord;
	vector<float> elnormal = element.normalcoord;
	vector<int> elindices = element.paintindices;
	unsigned int elvbo, elvbo2, elvao, elebo;

	//color
	glGenBuffers(1, &elvbo);
	glGenBuffers(1, &elvbo2);
	glGenBuffers(1, &elebo);
	glGenVertexArrays(1, &elvao);

	glBindVertexArray(elvao);
	glBindBuffer(GL_ARRAY_BUFFER, elvbo);
	glBufferData(GL_ARRAY_BUFFER, elcoord.size() * sizeof(float), &elcoord[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, elvbo2);
	glBufferData(GL_ARRAY_BUFFER, elnormal.size() * sizeof(float), &elnormal[0], GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, elindices.size() * sizeof(float), &elindices[0], GL_STATIC_DRAW);


	//-------------------------------------------------------------------------------------------------------------------------



	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	glfwSetCursorPosCallback(window, mouse_pos_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetKeyCallback(window, key_callback);


	Shader spline("shader/spline.vs", "shader/spline.fs");

	spline.use();

	//material information
	glm::vec3 diffusecolor;
	glm::vec3 specularcolor;
	float alpha;
	float shininess;

	//light information
	glm::vec3 lightcolor(1.0f);

	glm::vec3 diffuselight;
	glm::vec3 ambientlight;
	glm::vec3 specularlight;

	//direct light
	glm::vec3 directlightcolor = glm::vec3(0.0f);
	glm::vec3 direction(-1.0f, -1.0f, 0.0f);
	diffuselight = directlightcolor * glm::vec3(0.8f);
	ambientlight = directlightcolor * glm::vec3(0.2f);
	specularlight = directlightcolor * glm::vec3(0.7f);

	glUniform3fv(glGetUniformLocation(spline.ID, "direct.ambient"), 1, &ambientlight[0]);
	glUniform3fv(glGetUniformLocation(spline.ID, "direct.diffuse"), 1, &diffuselight[0]);
	glUniform3fv(glGetUniformLocation(spline.ID, "direct.specular"), 1, &specularlight[0]);
	glUniform3fv(glGetUniformLocation(spline.ID, "direct.direction"), 1, &direction[0]);


	//point light

	glm::vec3 pointlightcolor[] = {
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 1.0f),
		glm::vec3(1.0f, 0.6f, 0.6f)
	};

	glm::vec3 diffuselightcolor[] = {
	 pointlightcolor[0] * glm::vec3(0.8f),
	 pointlightcolor[1] * glm::vec3(0.8f),
	 pointlightcolor[2] * glm::vec3(0.8f)
	};

	glm::vec3 ambientlightcolor[] = {
	 pointlightcolor[0] * glm::vec3(0.3f),
	 pointlightcolor[1] * glm::vec3(0.3f),
	 pointlightcolor[2] * glm::vec3(0.0f)
	};

	glm::vec3 specularlightcolor[] = {
	 pointlightcolor[0] * glm::vec3(0.7f),
	 pointlightcolor[1] * glm::vec3(0.7f),
	 pointlightcolor[2] * glm::vec3(0.7f)
	};

	float constant = 1.0f;
	float linear = 0.09f;
	float quadratic = 0.032f;

	glm::vec3 pointlightPos[] = {
		glm::vec3(-90.0f,  100.0f,  -50.0f),
		glm::vec3(90.0f, 70.0f, 50.0f),
		glm::vec3(20.0f, 30.0f, -80.0f)
	};

	for (int i = 0; i < 3; i++) {
		string p;
		p = "point[" + to_string(i) + "]";
		glUniform3fv(glGetUniformLocation(spline.ID, (p + ".ambient").c_str()), 1, &ambientlightcolor[i][0]);
		glUniform3fv(glGetUniformLocation(spline.ID, (p + ".diffuse").c_str()), 1, &diffuselightcolor[i][0]);
		glUniform3fv(glGetUniformLocation(spline.ID, (p + ".specular").c_str()), 1, &specularlightcolor[i][0]);
		glUniform3fv(glGetUniformLocation(spline.ID, (p + ".position").c_str()), 1, &pointlightPos[i][0]);
		glUniform1f(glGetUniformLocation(spline.ID, (p + ".constant").c_str()), constant);
		glUniform1f(glGetUniformLocation(spline.ID, (p + ".linear").c_str()), linear);
		glUniform1f(glGetUniformLocation(spline.ID, (p + ".quadratic").c_str()), quadratic);
	}

	//-------------------------------------------------------------------------



	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//-------------------------------------------------------------------------
	//matrix
	mat4 i = glm::mat4(1.0f);
	mat4 tr = glm::translate(i, vec3(0, 0, 0));
	mat4 sc = glm::scale(i, vec3(1.0f));


	//waist(root)
	sc = glm::scale(i, vec3(2.5f, 0.5f, 2.5f));
	linkjoint waist(0, 1, 22, i, sc);

	//body
	tr = glm::translate(i, vec3(0, 13, 0));
	sc = glm::scale(i, vec3(2.5f, 1.0f, 2.5f));
	linkjoint body1(0, 1, 0, tr);
	linkjoint body2(0, 2, 0, i);
	linkjoint body3(0, 3, 1, i, sc);

	waist.insertnode(&body1);
	body1.insertnode(&body2);  
	body2.insertnode(&body3);

	//head
	tr = glm::translate(i, vec3(0, 22, 0));
	sc = glm::scale(i, vec3(2.0f, 0.5f, 2.0f));
	linkjoint head(0, 1, 1, tr, sc);
	body3.insertnode(&head);

	//larm
	tr = glm::translate(i, vec3(7, 22, 0));
	sc = glm::scale(i, vec3(1.0f, 0.7f, 1.0f));
	linkjoint larm1_1(0, 1, 0, tr);
	linkjoint larm1_2(0.1, 2, 0);
	linkjoint larm1_3(0.1, 3);

	linkjoint larm2(0, 3, 22, i, sc);


	body3.insertnode(&larm1_1);
	larm1_1.insertnode(&larm1_2);
	larm1_2.insertnode(&larm1_3);

	larm1_3.insertnode(&larm2);

	//lhand
	sc = glm::scale(i, vec3(1.3f, 0.4f, 1.3f));
	linkjoint lhand1(0, 1, 0);
	linkjoint lhand2(0.1, 2, 0);
	linkjoint lhand3(0.1, 3, 22, i, sc);

	larm2.insertnode(&lhand1);
	lhand1.insertnode(&lhand2);
	lhand2.insertnode(&lhand3);
	

	//rarm
	tr = glm::translate(i, vec3(-7, 22, 0));
	sc = glm::scale(i, vec3(1.0f, 0.7f, 1.0f));
	linkjoint rarm1_1(0, 1, 0, tr);
	linkjoint rarm1_2(-0.1, 2, 0);
	linkjoint rarm1_3(-0.1, 3);

	linkjoint rarm2(-0.1, 3, 22, i, sc);

	body3.insertnode(&rarm1_1);
	rarm1_1.insertnode(&rarm1_2);
	rarm1_2.insertnode(&rarm1_3);
	rarm1_3.insertnode(&rarm2);

	//rhand
	sc = glm::scale(i, vec3(1.3f, 0.4f, 1.3f));
	linkjoint rhand1(0, 1, 0);
	linkjoint rhand2(0.1, 2, 0);
	linkjoint rhand3(0.1, 3, 22, i, sc);

	rarm2.insertnode(&rhand1);
	rhand1.insertnode(&rhand2);
	rhand2.insertnode(&rhand3);


	//lleg
	tr = glm::translate(i, vec3(4, 0, 0));
	sc = glm::scale(i, vec3(1.0f, 1.3f, 1.0f));
	linkjoint lleg1_1(0.01, 1, 0, tr);
	linkjoint lleg1_2(0.01, 2, 0);
	linkjoint lleg1_3(0.1, 3, 22, i, sc);

	linkjoint lleg2(0.01, 1, 22);


	waist.insertnode(&lleg1_1);
	lleg1_1.insertnode(&lleg1_2);
	lleg1_2.insertnode(&lleg1_3);
	lleg1_3.insertnode(&lleg2);

	//lfoot
	tr = glm::translate(i, vec3(0, 0, 3));
	sc = glm::scale(i, vec3(1.0f, 0.4f, 1.8f));
	linkjoint lfoot1(0, 1, 0, tr);
	linkjoint lfoot2(0, 2, 0);
	linkjoint lfoot3(0, 3, 22, i, sc);

	lleg2.insertnode(&lfoot1);
	lfoot1.insertnode(&lfoot2);
	lfoot2.insertnode(&lfoot3);




	//rleg
	tr = glm::translate(i, vec3(-4, 0, 0));
	sc = glm::scale(i, vec3(1.0f, 1.3f, 1.0f));
	linkjoint rleg1_1(0.01, 1, 0, tr, sc);
	linkjoint rleg1_2(0.01, 2, 0, i, sc);
	linkjoint rleg1_3(-0.1, 3, 22, i, sc);

	linkjoint rleg2(0.01, 1, 22);

	waist.insertnode(&rleg1_1);
	rleg1_1.insertnode(&rleg1_2);
	rleg1_2.insertnode(&rleg1_3);
	rleg1_3.insertnode(&rleg2);

	//rfoot
	tr = glm::translate(i, vec3(0, 0, 3));
	sc = glm::scale(i, vec3(1.0f, 0.4f, 1.8f));
	linkjoint rfoot1(0, 1, 0, tr);
	linkjoint rfoot2(0, 2, 0);
	linkjoint rfoot3(0, 3, 22, i, sc);

	rleg2.insertnode(&rfoot1);
	rfoot1.insertnode(&rfoot2);
	rfoot2.insertnode(&rfoot3);


	//---------------------------------------------------------------------------------

	allnode.push_back(&lhand3);
	allnode.push_back(&rhand3);

	allnode.push_back(&lfoot3);
	allnode.push_back(&rfoot3);
	



	//render loop
	while (!glfwWindowShouldClose(window))//루프가 시작될 때마다 종료하도록 지시되었는지 확인, 그렇다면 true반환
	{
		float currenttime = glfwGetTime();
		processInput(window); //esc가 눌렸는지 루프마다 확인

		glClearColor(0.6f, 0.85f, 0.9f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::mat4 view = glm::mat4(1.0f);
		//view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
		view = glm::lookAt(cameraPos, centerofrot, cameraUp);

		glm::mat4 projection = glm::mat4(1.0f);
		projection = glm::perspective(glm::radians(fov), (float)800 / 600, 0.1f, 500.0f);

		glm::mat4 model = glm::mat4(1.0f);

		//---------------------------------------------------------------------------
		spline.use();
		glUniformMatrix4fv(glGetUniformLocation(spline.ID, "view"), 1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(glGetUniformLocation(spline.ID, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
		glUniform3fv(glGetUniformLocation(spline.ID, "viewPos"), 1, &cameraPos[0]);
		glUniform1f(glGetUniformLocation(spline.ID, "shadow"), 1.0f);

		//---------------------------------------------------------------------------


		//sphere--------------------------------------------------------------------

		//material information
		glm::vec3 diffusecolor = glm::vec3(4.0f, 3.0f, 3.0f);
		glm::vec3 specularcolor = glm::vec3(0.0f, 0.0f, 0.0f);
		alpha = 1;
		shininess = 64;
		glUniform1f(glGetUniformLocation(spline.ID, "material.shininess"), shininess);
		glUniform1f(glGetUniformLocation(spline.ID, "material.alpha"), alpha);
		glUniform3fv(glGetUniformLocation(spline.ID, "material.diffuse"), 1, &diffusecolor[0]);
		glUniform3fv(glGetUniformLocation(spline.ID, "material.specular"), 1, &specularcolor[0]);
		
		glBindVertexArray(elvao);
		diffusecolor = glm::vec3(3.0f, 3.0f, 5.0f);
		glUniform3fv(glGetUniformLocation(spline.ID, "material.diffuse"), 1, &diffusecolor[0]);

		model = mat4(1.0f);
		mat4 m = translate(model, goalpos);
		m = m* toMat4(goalori);
		m = m * allnode[jointnum]->scmodel;
		glUniformMatrix4fv(glGetUniformLocation(spline.ID, "model"), 1, GL_FALSE, glm::value_ptr(m));
		glDrawElements(GL_TRIANGLES, ind.size(), GL_UNSIGNED_INT, NULL);


		//-------------------------------------------------------------------------

		//element--------------------------------------------------------------------
		glBindVertexArray(elvao);

		//material information
		diffusecolor = glm::vec3(2.0f, 2.0f, 2.0f);
		specularcolor = glm::vec3(1.0f, 1.0f, 1.0f);
		float alpha = 1;
		float shininess = 64;
		glUniform1f(glGetUniformLocation(spline.ID, "material.shininess"), shininess);
		glUniform1f(glGetUniformLocation(spline.ID, "material.alpha"), alpha);
		glUniform3fv(glGetUniformLocation(spline.ID, "material.diffuse"), 1, &diffusecolor[0]);
		glUniform3fv(glGetUniformLocation(spline.ID, "material.specular"), 1, &specularcolor[0]);
		glUniform3fv(glGetUniformLocation(spline.ID, "viewPos"), 1, &cameraPos[0]);

		
		if (movestart) {
			movenode(allnode[jointnum]);
		}

		drawnode(&waist, spline, elindices.size());

		glfwSwapBuffers(window);
		glfwPollEvents();
	
	}

	glfwTerminate();
	return 0;
}





void framebuffer_size_callback(GLFWwindow* window, int width, int height) //창의 크기를 조정하는 함수 (객체, 너비, 높이)
{
	glViewport(0, 0, width, height); //윈도우의 왼쪽 아래 모서리의 위치, 너비와 높이의 픽셀 (-1~1 사이의 좌표값을 원하는 값의 픽셀스케일로 변환)
}


void mouseclick_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
		swt = -swt + 1;
}

int temp = 1;
glm::vec3 firstvec;
glm::vec3 secondvec;
glm::vec3 xvec(1, 0, 0);
glm::vec3 yvec(0, 1, 0);
glm::quat rotate_quat(1, 0, 0, 0);
glm::vec3 firstcamera;

glm::vec3 posvec2;
float lastX, lastY;
bool firstmouse = true;

float yaw = -90.0f;
float pitch = 0.0f;




void mouse_pos_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (!movemode) {
		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
		{
			posvec.x = (xpos - 400);
			posvec.y = (300 - ypos);
			posvec.z = 0;

			if (temp == 1) {
				firstvec = cameraPos - centerofrot + xvec * posvec.x + yvec * posvec.y;
				temp = 0;
			}

			secondvec = cameraPos - centerofrot + xvec * posvec.x + yvec * posvec.y;

			if (firstvec != secondvec) {
				glm::vec3 cross = glm::cross(secondvec, firstvec);
				rotate_axis = glm::normalize(cross);
				rotate_theta = atan2(glm::length(cross), glm::dot(firstvec, secondvec));
				rotate_quat = glm::angleAxis(rotate_theta, rotate_axis);
				cameraPos = rotate_quat * (cameraPos - centerofrot) * glm::inverse(rotate_quat) + centerofrot;
				xvec = rotate_quat * xvec * glm::inverse(rotate_quat);
				yvec = rotate_quat * yvec * glm::inverse(rotate_quat);
				cameraUp = yvec;
				temp = 1;
			}
		}

		else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
		{
			temp = 1;
		}

		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
			if (firstmouse)
			{
				lastX = xpos;
				lastY = ypos;
				firstmouse = false;
				firstvec = cameraPos - centerofrot;
			}
			float xoffset = -(xpos - lastX) * glm::length(cameraPos) * 0.0013;
			float yoffset = -(lastY - ypos) * glm::length(cameraPos) * 0.0013;
			lastX = xpos;
			lastY = ypos;

			posvec2 = xoffset * xvec + yoffset * yvec;
			cameraPos += posvec2;
			centerofrot += posvec2;
		}

		else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE)
		{
			firstmouse = true;
		}
	}
	else {
		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
			if (firstmouse)
			{
				lastX = xpos;
				lastY = ypos;
				firstmouse = false;
				firstvec = cameraPos - centerofrot;
			}

			float xoffset = (xpos - lastX) * glm::length(cameraPos) * 0.0015;
			float yoffset = (lastY - ypos) * glm::length(cameraPos) * 0.0015;
			lastX = xpos;
			lastY = ypos;

			posvec2 = xoffset * xvec + yoffset * yvec;
			goalpos += posvec2;
		}
		else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
		{
			firstmouse = true;
		}
	}

}


void processInput(GLFWwindow* window)
{
	glm::vec3 axis(0, 1, 0);
	float theta = 0.02f;
	glm::quat quat = glm::angleAxis(theta, axis);

	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == 1)
		glfwSetWindowShouldClose(window, true);

	float cameraSpeed = 1.0f * 0.02f;

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) //dolly in
		cameraPos -= cameraSpeed * (cameraPos - centerofrot);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) //dolly out
		cameraPos += cameraSpeed * (cameraPos - centerofrot);

}




void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_C && action == GLFW_PRESS) {
		movemode = !movemode;
	}
	if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
		movestart = !movestart;
	}
	if (key == GLFW_KEY_LEFT && action == GLFW_PRESS) {
		jointnum = (jointnum + 1) % allnode.size();
		movestart = false;
	}

	if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS) {
		jointnum -= 1;
		if (jointnum == -1)
			jointnum = allnode.size() - 1;
		movestart = false;
	}

}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mode)
{


}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	if (fov >= 1.0f && fov <= 45.0f)
		fov -= yoffset;
	if (fov <= 1.0f)
		fov = 1.0f;
	if (fov >= 45.0f)
		fov = 45.0f;
}

void drawnode(linkjoint *root, Shader spline, int elindices) {
	mat4 m;
	m = root->modelmat();
	if (root->length) {
		if(root == allnode[jointnum]){
			vec3 diffusecolor = glm::vec3(3.0f, 3.0f, 5.0f);
			glUniform3fv(glGetUniformLocation(spline.ID, "material.diffuse"), 1, &diffusecolor[0]);
		}
		else {
			vec3 diffusecolor = glm::vec3(2.0f, 2.0f, 2.0f);
			glUniform3fv(glGetUniformLocation(spline.ID, "material.diffuse"), 1, &diffusecolor[0]);
		}
		glUniformMatrix4fv(glGetUniformLocation(spline.ID, "model"), 1, GL_FALSE, glm::value_ptr(m));
		glDrawElements(GL_TRIANGLES, elindices, GL_UNSIGNED_INT, NULL);
	}
	for (linkjoint* a : root->node) {
		drawnode(a, spline, elindices);
	}
}



void findinverse(float** a) {//gauss jordan elimination
	float temp;
	float a2[6][6];
	float b[6][6];
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++) {
			a2[i][j] = a[i][j];
			if (i == j)
				b[i][j] = 1;
			else
				b[i][j] = 0;
		}
	for (int i = 0; i < 6; i++) {
		temp = a2[i][i];
		for (int j = 0; j < 6; j++) {
			a2[i][j] /= temp;
			b[i][j] /= temp;
		}
		for (int k = 0; k < 6; k++) {
			temp = a2[k][i];
			for (int l = 0; l < 6; l++) {
				if (i == k) {
					break;
				}
				a2[k][l] -= a2[i][l] * temp;
				b[k][l] -= b[i][l] * temp;
			}
		}
	}

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			a[i][j] = b[i][j];
		}
	}
}

void multiplymat(float** a, float** b, int n1, int n2, int n3, float** c) {

	for (int i = 0; i < n1; i++) {
		for (int j = 0; j < n3; j++) {
			c[i][j] = 0;
			for (int k = 0; k < n2; k++) {
				c[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}

void transposemat(float** a, int n1, int n2, float** a2) {
	for (int i = 0; i < n1; i++) {
		for (int j = 0; j < n2; j++) {
			a2[j][i] = a[i][j];
		}
	}
}

float determinant(float** a, int n) {
	float det = 0;

	float** a2 = nullptr;
	a2 = new float* [6];
	for (int i = 0; i < 6; i++) {
		a2[i] = new float[6];
	}

	if (n == 2) {
		return ((a[0][0] * a[1][1]) - (a[1][0] * a[0][1]));
	}
	else {
		for (int x = 0; x < n; x++) {
			int subi = 0;
			for (int i = 1; i < n; i++) {
				int subj = 0;
				for (int j = 0; j < n; j++) {
					if (j == x)
						continue;
					a2[subi][subj] = a[i][j];
					subj++;
				}
				subi++;
			}
			det = det + (pow(-1, x) * a[0][x] * determinant(a2, n - 1));
		}
	}
	return det;
}

void pinverse(float **a, int n, float **b) {
	float** a2 = nullptr;
	a2 = new float* [n];
	for (int i = 0; i < n; i++) {
		a2[i] = new float[6];
	}

	float** a3 = nullptr;
	a3 = new float* [6];
	for (int i = 0; i < 6; i++) {
		a3[i] = new float[6];
	}

	transposemat(a, 6, n, a2);
	multiplymat(a, a2, 6, n, 6, a3);

	/*
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			if (i == j) {
				a3[i][j] += 0.01;
			}
		}
	}
	*/

	findinverse(a3);
	multiplymat(a2, a3, n, 6, 6, b);

	for (int i = 0; i < n; i++) {
		delete[] a2[i];
	}
	delete[] a2;

	for (int i = 0; i < 6; i++) {
		delete[] a3[i];
	}
	delete[] a3;
}




void movenode(linkjoint* node) {


	vec3 endpoint = vec3(node->modelmat() * vec4(0, 0, 0, 1.0f));
	
	mat4 endrot = node->modelmat();

	/*
	float thetax = -atan2(endrot[2][1], endrot[2][2]);
	float c = sqrt(endrot[0][0] * endrot[0][0] + endrot[1][0] * endrot[1][0]);
	float thetay = atan2(-endrot[2][0], c);
	float s1 = sin(thetax), c1 = cos(thetax);
	float thetaz = atan2(s1 * endrot[0][2] - c1 * endrot[0][1], c1 * endrot[1][1] - s1 * endrot[1][2]);
	cout << thetax << " " << thetay << " " << thetaz << endl;
	*/

	quat endori = quat_cast(endrot);
	quat diffquat = goalori * inverse(endori);
	float diffori = length(diffquat);


	//cout << (goalori - endori).x << " " << (goalori - endori).y << " " << (goalori - endori).z << endl;
	if (length(goalpos - endpoint) > 1 || diffori>1.1 || diffori<0.9) {

		//float len = length(goalpos - endpoint);
		//cout << len << " ";
		//cout << diffori << endl;

		vector<float> theta;
		vector<linkjoint*> chnode;

		theta.push_back(node->theta);
		chnode.push_back(node);

		linkjoint* a = node->uppernode;
		while (a) {
			theta.push_back(a->theta);
			chnode.push_back(a);
			a = a->uppernode;
		}

		theta.pop_back();
		chnode.pop_back();


		int size = theta.size();


		float** jacob = new float* [6];
		for (int i = 0; i < 6; i++) {
			jacob[i] = new float[size];
		}

		float** pjacob = new float* [size];
		for (int i = 0; i < size; i++) {
			pjacob[i] = new float[6];
		}





		for (int i = 0; i < size; i++) {
			jacob[0][i] = chnode[size - i - 1]->pvec(endpoint).x;
			jacob[1][i] = chnode[size - i - 1]->pvec(endpoint).y;
			jacob[2][i] = chnode[size - i - 1]->pvec(endpoint).z;

			jacob[3][i] = chnode[size - i - 1]->axis.x;
			jacob[4][i] = chnode[size - i - 1]->axis.y;
			jacob[5][i] = chnode[size - i - 1]->axis.z;
		}


		pinverse(jacob, size, pjacob);

		/*
		float** error = new float* [6];
		for (int i = 0; i < 6; i++) {
			error[i] = new float[6];
		}
		multiplymat(jacob, pjacob, 6, size, 6, error);
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 6; j++) {
				if (i == j) {
					error[i][j] = 1 - error[i][j];
				}
				else {
					error[i][j] = -error[i][j];
				}
			}
		}
		cout << determinant(error, 6) << endl;
		*/


		/*
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < size; j++) {
				cout << jacob[i][j] << " ";
			}
			cout << endl;
		}*/

		vec3 goalvec = (goalpos - endpoint);
		vec3 orivec = angle(diffquat) * axis(diffquat) ;

		//cout << endpoint.x << " " << endpoint.y << " " << endpoint.z << endl;
		//cout << goalvec.x << " " << goalvec.y << " " << goalvec.z << endl;


		float endeffector[6] = { goalvec.x, goalvec.y, goalvec.z, orivec.x, orivec.y, orivec.z};

		//cout << length(goalpos - endpoint) << endl;
		//cout << endeffector[0] << " " << endeffector[1] << " " << endeffector[2] << endl;
		
		for (int i = 0; i < size; i++) {
			for (int j = 0; j < 6; j++) {
				theta[size - i - 1] += 0.1 * pjacob[i][j] * endeffector[j] * (1 + i) / (size);
			}
		}

		for (int i = 0; i < size; i++) {
			chnode[i]->changetheta(theta[i]);
		}


		for (int i = 0; i < size; i++) {
			delete[] pjacob[i];
		}
		delete[] pjacob;

		for (int i = 0; i < 6; i++) {
			delete[] jacob[i];
		}
		delete[] jacob;

	}
}