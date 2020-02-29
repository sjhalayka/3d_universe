


#ifndef main_H
#define main_H



#define _CRT_SECURE_NO_WARNINGS 1


#include "uv_camera.h"
#include "custom_math.h"
using namespace custom_math;


#include <fstream>
#include <ios>
#include <map>
#include <vector>
#include <sstream>
#include <algorithm>
using namespace std;




#include <opencv2/opencv.hpp>
using namespace cv;
#pragma comment(lib, "opencv_world411.lib")



vector_3 background_colour(1.0, 1.0, 1.0);

float outline_width = 3.0;
static const float outline_colour[] = {0.0, 0.0, 0.0};

bool draw_outline = true;
bool draw_axis = true;
bool draw_control_list = true;
bool screenshot_mode = false;

uv_camera main_camera;

GLint win_id = 0;
GLint win_x = 800, win_y = 600;
float camera_w = 3;
float camera_fov = 45;
float camera_x_transform = 0;
float camera_y_transform = 0;
double u_spacer = 0.01;
double v_spacer = 0.5*u_spacer;
double w_spacer = 0.1;
double camera_near = 0.0001;
double camera_far = 100.0;

GLUquadricObj* glu_obj = gluNewQuadric(); // Probably should delete this before app exit... :)

bool lmb_down = false;
bool mmb_down = false;
bool rmb_down = false;
int mouse_x = 0;
int mouse_y = 0;

void idle_func(void);
void init_opengl(const int &width, const int &height);
void reshape_func(int width, int height);
void display_func(void);
void keyboard_func(unsigned char key, int x, int y);
void mouse_func(int button, int state, int x, int y);
void motion_func(int x, int y);
void passive_motion_func(int x, int y);
void draw_objects(bool disable_colouring = false);

vector<vector_3> vertices;
vector<indexed_triangle> triangles;
map<size_t, vector<size_t> > tri_neighbours;


vector<float> get_tri_edge_lengths(const indexed_triangle& it)
{
	vector<float> ret;

	vector_3 v0 = vertices[it.vertex_indices[0]];
	vector_3 v1 = vertices[it.vertex_indices[1]];
	vector_3 v2 = vertices[it.vertex_indices[2]];

	ret.push_back(static_cast<float>(d_3(v0, v1)));
	ret.push_back(static_cast<float>(d_3(v0, v2)));
	ret.push_back(static_cast<float>(d_3(v1, v2)));

	return ret;
}



void write_histogram(vector<float> input_vec, const char* const filename)
{

	float input_vec_max_val = 0;

	for (size_t i = 0; i < input_vec.size(); i++)
	{
		if (input_vec[i] > input_vec_max_val)
			input_vec_max_val = input_vec[i];
	}

	for (size_t i = 0; i < input_vec.size(); i++)
	{
		input_vec[i] /= input_vec_max_val;
		input_vec[i] *= 255.0f;
		input_vec[i] = floorf(input_vec[i]);
	}

	Mat data(1, static_cast<int>(input_vec.size()), CV_8UC1, Scalar(0));

	for (size_t i = 0; i < input_vec.size(); i++)
		data.at<unsigned char>(0, static_cast<int>(i)) = static_cast<unsigned char>(input_vec[i]);

	int histSize = 256;
	float range[] = { 0, 256 }; //the upper boundary is exclusive
	const float* histRange = { range };
	bool uniform = true, accumulate = false;
	Mat hist;
	calcHist(&data, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

	int hist_w = 600, hist_h = 600;
	int bin_w = cvRound((double)hist_w / histSize);
	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(255, 255, 255));
	normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

	float largest_hist = 0;
	float largest_hist_j = 0;

	for (int j = 0; j < hist.rows; j++)
	{
		for (int i = 0; i < hist.cols; i++)
		{
			if (hist.at<float>(j, i) > largest_hist)
			{
				largest_hist = hist.at<float>(j, i);
				largest_hist_j = static_cast<float>(j);
			}
		}
	}

	for (int i = 1; i < histSize; i++)
	{
		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(hist.at<float>(i))),
			Scalar(0, 0, 0), 1, 8, 0);
	}

	float factor = static_cast<float>(largest_hist_j * bin_w) / static_cast<float>(histImage.cols - 1);
	cout << "max value:  " << input_vec_max_val << endl;
	cout << "peak value: " << input_vec_max_val * factor << endl;


	//circle(histImage, Point(static_cast<int>(largest_hist_j)* bin_w, 0), 2, Scalar(255, 127, 0), 2);

	imwrite(filename, histImage);
	//imshow("calcHist Demo", histImage);
	//waitKey();
}




void get_sorted_lines_from_triangle(size_t tri_index, vector<indexed_line_segment>& lines)
{
	lines.clear();

	indexed_line_segment ils;
	vector<size_t> sorted_vertex_indices;

	// do edge 0
	ils.vertex_indices[0] = triangles[tri_index].vertex_indices[0];
	ils.vertex_indices[1] = triangles[tri_index].vertex_indices[1];

	sorted_vertex_indices.clear();
	sorted_vertex_indices.push_back(ils.vertex_indices[0]);
	sorted_vertex_indices.push_back(ils.vertex_indices[1]);

	sort(sorted_vertex_indices.begin(), sorted_vertex_indices.end());

	ils.vertex_indices[0] = sorted_vertex_indices[0];
	ils.vertex_indices[1] = sorted_vertex_indices[1];

	lines.push_back(ils);

	// do edge 1
	ils.vertex_indices[0] = triangles[tri_index].vertex_indices[0];
	ils.vertex_indices[1] = triangles[tri_index].vertex_indices[2];

	sorted_vertex_indices.clear();
	sorted_vertex_indices.push_back(ils.vertex_indices[0]);
	sorted_vertex_indices.push_back(ils.vertex_indices[1]);

	sort(sorted_vertex_indices.begin(), sorted_vertex_indices.end());

	ils.vertex_indices[0] = sorted_vertex_indices[0];
	ils.vertex_indices[1] = sorted_vertex_indices[1];

	lines.push_back(ils);

	// do edge 2
	ils.vertex_indices[0] = triangles[tri_index].vertex_indices[1];
	ils.vertex_indices[1] = triangles[tri_index].vertex_indices[2];

	sorted_vertex_indices.clear();
	sorted_vertex_indices.push_back(ils.vertex_indices[0]);
	sorted_vertex_indices.push_back(ils.vertex_indices[1]);

	sort(sorted_vertex_indices.begin(), sorted_vertex_indices.end());

	ils.vertex_indices[0] = sorted_vertex_indices[0];
	ils.vertex_indices[1] = sorted_vertex_indices[1];

	lines.push_back(ils);
}


void get_all_tri_neighbours(void)
{
	tri_neighbours.clear();

	vector<size_t> default_lookup;

	for (size_t i = 0; i < triangles.size(); i++)
		tri_neighbours[i] = default_lookup;

	cout << "Enumerating shared faces" << endl;

	map<indexed_line_segment, vector<size_t> > neighbours;

	for (size_t i = 0; i < triangles.size(); i++)
	{
		if(i % 100 == 0)
		cout << i + 1 << " of " << triangles.size() << endl;

		vector<indexed_line_segment> lines;

		get_sorted_lines_from_triangle(i, lines);

		for (size_t j = 0; j < lines.size(); j++)
			neighbours[lines[j]].push_back(i);
	}

	cout << "Processing shared faces" << endl;

	size_t count = 0;

	for (map<indexed_line_segment, vector<size_t> >::const_iterator ci = neighbours.begin(); ci != neighbours.end(); ci++)
	{
		if (count % 100 == 0)
			cout << count + 1 << " of " << neighbours.size() << endl;

		count++;

		tri_neighbours[ci->second[0]].push_back(ci->second[1]);
		tri_neighbours[ci->second[1]].push_back(ci->second[0]);
	}
}




void get_vertices_and_triangles(const size_t num_vertices)
{
	vertices.clear();
	triangles.clear();

	ostringstream oss;
	oss << num_vertices;

	// rbox s 10 D3 | qconvex i Qt

	string rbox_cmdline = "rbox s ";
	rbox_cmdline += oss.str();
	rbox_cmdline += " D3 > vertices.txt";

	system(rbox_cmdline.c_str());

	ifstream rbox_file("vertices.txt");

	string rbox_cmdline_short;

	string line;
	getline(rbox_file, rbox_cmdline_short);
	getline(rbox_file, line);

	for (size_t i = 0; i < num_vertices; i++)
	{
		getline(rbox_file, line);

		istringstream iss(line);

		vector_3 v;
		iss >> v.x;
		iss >> v.y;
		iss >> v.z;

		vertices.push_back(v);
	}
	
	rbox_file.close();


	ofstream rbox_file_out("vertices.txt");

	rbox_file_out << rbox_cmdline_short << endl;
	rbox_file_out << vertices.size() << endl;

	// Process vertices here
	//attract_vertices(1000);

	for (size_t i = 0; i < vertices.size(); i++)
		rbox_file_out << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << endl;

	rbox_file_out.close();

	string qconvex_cmdline = "\"qconvex i Qt < vertices.txt\" > triangles.txt";

	system(qconvex_cmdline.c_str());

	ifstream qconvex_file("triangles.txt");

	getline(qconvex_file, line);
	
	istringstream iss_triangle_count(line);

	size_t num_triangles = 0;
	iss_triangle_count >> num_triangles;

	for (size_t i = 0; i < num_triangles; i++)
	{
		getline(qconvex_file, line);

		istringstream iss(line);

		indexed_triangle t;
		iss >> t.vertex_indices[0];
		iss >> t.vertex_indices[1];
		iss >> t.vertex_indices[2];

		triangles.push_back(t);
	}

	cout << "Vertex count:   " << vertices.size() << endl;
	cout << "Triangle count: " << triangles.size() << endl;

	get_all_tri_neighbours();
}










 
// TODO: fix camera bug where portrait mode crashes.
void take_screenshot(size_t num_cams_wide, const char *filename, const bool reverse_rows = false)
{
	screenshot_mode = true;

	// Set up Targa TGA image data.
	unsigned char  idlength = 0;
	unsigned char  colourmaptype = 0;
	unsigned char  datatypecode = 2;
	unsigned short int colourmaporigin = 0;
	unsigned short int colourmaplength = 0;
	unsigned char  colourmapdepth = 0;
	unsigned short int x_origin = 0;
	unsigned short int y_origin = 0;

	cout << "Image size: " << static_cast<size_t>(win_x)*num_cams_wide << "x" << static_cast<size_t>(win_y)*num_cams_wide << " pixels" << endl;

	if (static_cast<size_t>(win_x)*num_cams_wide > static_cast<unsigned short>(-1) ||
		static_cast<size_t>(win_y)*num_cams_wide > static_cast<unsigned short>(-1))
	{
		cout << "Image too large. Maximum width and height is " << static_cast<unsigned short>(-1) << endl;
		return;
	}

	unsigned short int px = win_x*static_cast<unsigned short>(num_cams_wide);
	unsigned short int py = win_y*static_cast<unsigned short>(num_cams_wide);
	unsigned char  bitsperpixel = 24;
	unsigned char  imagedescriptor = 0;
	vector<char> idstring;

	size_t num_bytes = 3*px*py;
	vector<unsigned char> pixel_data(num_bytes);

	// Adjust some parameters for large screen format.
	bool temp_draw_control_list = draw_control_list;
	draw_control_list = false;

	float temp_outline_width = outline_width;
	outline_width = 12;

	vector<unsigned char> fbpixels(3*win_x*win_y);

	const size_t total_cams = num_cams_wide * num_cams_wide;
	size_t cam_count = 0;
	// Loop through subcameras.
	for(size_t cam_num_x = 0; cam_num_x < num_cams_wide; cam_num_x++)
	{
		for(size_t cam_num_y = 0; cam_num_y < num_cams_wide; cam_num_y++)
		{
			cout << "Camera: " << cam_count + 1 << " of " << total_cams << endl;

			// Set up camera, draw, then copy the frame buffer.
			main_camera.Set_Large_Screenshot(num_cams_wide, cam_num_x, cam_num_y);
			display_func();
			glReadPixels(0, 0, win_x, win_y, GL_RGB, GL_UNSIGNED_BYTE, &fbpixels[0]);

			// Copy pixels to large image.
			for(GLint i = 0; i < win_x; i++)
			{
				for(GLint j = 0; j < win_y; j++)
				{
					size_t fb_index = 3*(j*win_x + i);

					size_t screenshot_x = cam_num_x*win_x + i;
					size_t screenshot_y = cam_num_y*win_y + j;
					size_t screenshot_index = 3*(screenshot_y*(win_x*num_cams_wide) + screenshot_x);

					pixel_data[screenshot_index] = fbpixels[fb_index + 2];
					pixel_data[screenshot_index + 1] = fbpixels[fb_index + 1];
					pixel_data[screenshot_index + 2] = fbpixels[fb_index ];
				}
			}

			cam_count++;
		}

	}

	screenshot_mode = false;

	// Restore the parameters.
	draw_control_list = temp_draw_control_list;
	outline_width = temp_outline_width;
	main_camera.Set();

	// Write Targa TGA file to disk.
	ofstream out(filename, ios::binary);

	if(!out.is_open())
	{
		cout << "Failed to open TGA file for writing: " << filename << endl;
		return;
	}

	out.write(reinterpret_cast<char *>(&idlength), 1);
	out.write(reinterpret_cast<char *>(&colourmaptype), 1);
	out.write(reinterpret_cast<char *>(&datatypecode), 1);
	out.write(reinterpret_cast<char *>(&colourmaporigin), 2);
	out.write(reinterpret_cast<char *>(&colourmaplength), 2);
	out.write(reinterpret_cast<char *>(&colourmapdepth), 1);
	out.write(reinterpret_cast<char *>(&x_origin), 2);
	out.write(reinterpret_cast<char *>(&y_origin), 2);
	out.write(reinterpret_cast<char *>(&px), 2);
	out.write(reinterpret_cast<char *>(&py), 2);
	out.write(reinterpret_cast<char *>(&bitsperpixel), 1);
	out.write(reinterpret_cast<char *>(&imagedescriptor), 1);

	out.write(reinterpret_cast<char *>(&pixel_data[0]), num_bytes);

}

void idle_func(void)
{
	glutPostRedisplay();
}




void init_opengl(const int &width, const int &height)
{
	win_x = width;
	win_y = height;

	if(win_x < 1)
		win_x = 1;

	if(win_y < 1)
		win_y = 1;

	glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_ALPHA|GLUT_DEPTH);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(win_x, win_y);
	win_id = glutCreateWindow("2D v4");

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDepthMask(GL_TRUE);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_POLYGON_SMOOTH);

	float light_colour[] = {0.9f, 0.9f, 0.9f, 1.0f};
	float light0_position[] = {0.0f, 0.0f, 1.0f, 0.0f };
	float light1_position[] = {0.0f, 0.0f, -1.0f, 0.0f };
	float light2_position[] = {1.0f, 0.0f, 0.0f, 0.0f };
	float light3_position[] = {-1.0f, 0.0f, 0.0f, 0.0f };

	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_colour);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_colour);
	glLightfv(GL_LIGHT2, GL_POSITION, light2_position);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, light_colour);
	glLightfv(GL_LIGHT3, GL_POSITION, light3_position);
	glLightfv(GL_LIGHT3, GL_DIFFUSE, light_colour);

	float light_colour2[] = {0.5f, 0.5f, 0.5f, 1.0f};
	float light4_position[] = {0.0f, 1.0f, 0.0f, 0.0f };
	float light5_position[] = {0.0f, -1.0f, 0.0f, 0.0f };

	glLightfv(GL_LIGHT4, GL_POSITION, light4_position);
	glLightfv(GL_LIGHT4, GL_DIFFUSE, light_colour2);
	glLightfv(GL_LIGHT5, GL_POSITION, light5_position);
	glLightfv(GL_LIGHT5, GL_DIFFUSE, light_colour2);

	glClearColor(static_cast<float>(background_colour.x), static_cast<float>(background_colour.y), static_cast<float>(background_colour.z), 1.0f);
	glClearDepth(1.0f);

	main_camera.Set(0, 0, camera_w, camera_fov, win_x, win_y, camera_near, camera_far);

}

void reshape_func(int width, int height)
{
	win_x = width;
	win_y = height;

	if(win_x < 1)
		win_x = 1;

	if(win_y < 1)
		win_y = 1;

	glutSetWindow(win_id);
	glutReshapeWindow(win_x, win_y);
	glViewport(0, 0, win_x, win_y);

	main_camera.Set(main_camera.u, main_camera.v, main_camera.w, main_camera.fov, win_x, win_y, camera_near, camera_far);
}

// Text drawing code originally from "GLUT Tutorial -- Bitmap Fonts and Orthogonal Projections" by A R Fernandes
void render_string(int x, const int y, void *font, const string &text)
{
	for(size_t i = 0; i < text.length(); i++)
	{
		glRasterPos2i(x, y);
		glutBitmapCharacter(font, text[i]);
		x += glutBitmapWidth(font, text[i]) + 1;
	}
}
// End text drawing code.

void display_func(void)
{
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	if(true == draw_outline)
	{
		glDisable(GL_DEPTH_TEST);

		// Draw outline code from NeHe lesson 37:
		// http://nehe.gamedev.net/data/lessons/lesson.asp?lesson=37
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glLineWidth(outline_width);
		glCullFace(GL_BACK);
		glPolygonMode(GL_FRONT, GL_LINE);
		glColor3fv(&outline_colour[0]);

		draw_objects(true);


		glPopAttrib();
		// End draw outline code.

		glEnable(GL_DEPTH_TEST);
	}


	// Draw the model's components using OpenGL/GLUT primitives.
	draw_objects();




	if(true == draw_outline)
	{
		// Draw outline code from NeHe lesson 37:
		// http://nehe.gamedev.net/data/lessons/lesson.asp?lesson=37
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glLineWidth(outline_width);
		glCullFace(GL_FRONT);
		glPolygonMode(GL_BACK, GL_LINE);

		draw_objects(true);

		glPopAttrib();
		// End draw outline code.
	}

	if(true == draw_control_list)
	{
		// Text drawing code originally from "GLUT Tutorial -- Bitmap Fonts and Orthogonal Projections" by A R Fernandes
		// http://www.lighthouse3d.com/opengl/glut/index.php?bmpfontortho
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluOrtho2D(0, win_x, 0, win_y);
		glScalef(1, -1, 1); // Neat. :)
		glTranslatef(0, -static_cast<float>(win_y), 0); // Neat. :)
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glColor3f(0, 0, 0);

		int break_size = 13;
		int start = 20;

		render_string(10, start, GLUT_BITMAP_HELVETICA_12, string("Keyboard controls:"));

		render_string(10, start + 1*break_size, GLUT_BITMAP_HELVETICA_10, string("H: Draw outlines"));
		render_string(10, start + 2*break_size, GLUT_BITMAP_HELVETICA_10, string("J: Draw axis"));
		render_string(10, start + 3*break_size, GLUT_BITMAP_HELVETICA_10, string("K: Draw this list"));
		render_string(10, start + 4*break_size, GLUT_BITMAP_HELVETICA_10, string("L: Take screenshot"));


		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		//glMatrixMode(GL_MODELVIEW);
		// End text drawing code.
	}

	if (false == screenshot_mode)
	{
		glFlush();
		glutSwapBuffers();
	}
}

void keyboard_func(unsigned char key, int x, int y)
{
	switch(tolower(key))
	{
	case 'h':
		{
			draw_outline = !draw_outline;
			break;
		}
	case 'j':
		{
			draw_axis = !draw_axis;
			break;
		}
	case 'k':
		{
			draw_control_list = !draw_control_list;
			break;
		}
	case 'l':
		{
			take_screenshot(4, "screenshot.tga");
			break;
		}

	default:
		break;
	}
}

void mouse_func(int button, int state, int x, int y)
{
	if(GLUT_LEFT_BUTTON == button)
	{
		if(GLUT_DOWN == state)
			lmb_down = true;
		else
			lmb_down = false;
	}
	else if(GLUT_MIDDLE_BUTTON == button)
	{
		if(GLUT_DOWN == state)
			mmb_down = true;
		else
			mmb_down = false;
	}
	else if(GLUT_RIGHT_BUTTON == button)
	{
		if(GLUT_DOWN == state)
			rmb_down = true;
		else
			rmb_down = false;
	}
}

void motion_func(int x, int y)
{
	int prev_mouse_x = mouse_x;
	int prev_mouse_y = mouse_y;

	mouse_x = x;
	mouse_y = y;

	int mouse_delta_x = mouse_x - prev_mouse_x;
	int mouse_delta_y = prev_mouse_y - mouse_y;

	if(true == lmb_down && (0 != mouse_delta_x || 0 != mouse_delta_y))
	{
		main_camera.u -= static_cast<float>(mouse_delta_y)*u_spacer;
		main_camera.v += static_cast<float>(mouse_delta_x)*v_spacer;
	}
	else if(true == rmb_down && (0 != mouse_delta_y))
	{
		main_camera.w -= static_cast<float>(mouse_delta_y)*w_spacer;
	}

	main_camera.Set(); // Calculate new camera vectors.
}

void passive_motion_func(int x, int y)
{
	mouse_x = x;
	mouse_y = y;
}

// This render mode won't apply to a curved 3D space.
void draw_objects(bool disable_colouring)
{
	if(false == disable_colouring)
	{
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		glEnable(GL_LIGHT1);
		glEnable(GL_LIGHT2);
		glEnable(GL_LIGHT3);
		glEnable(GL_LIGHT4);
		glEnable(GL_LIGHT5);
	}
	else
	{
		glColor3f(0.0f, 0.0f, 0.0f);
		glDisable(GL_LIGHTING);
	}

	static const float rad_to_deg = 180.0f/static_cast<float>(pi);

	glPushMatrix();

	glPointSize(5.0f);

	glBegin(GL_POINTS);
	glColor3f(1.0f, 0.5f, 0.0f);

	for (size_t i = 0; i < vertices.size(); i++)
	{
		vector_3 v3 = vertices[i];

		double x1 = vertices[i].x;
		double x2 = vertices[i].y;
		double x3 = vertices[i].z;

		glVertex3d(x1, x2, x3);
	}


	glEnd();

	for (size_t i = 0; i < triangles.size(); i++)
	{
		vector_3 a = vertices[triangles[i].vertex_indices[2]];
		vector_3 b = vertices[triangles[i].vertex_indices[1]];
		vector_3 c = vertices[triangles[i].vertex_indices[0]];

		glLineWidth(1.0);

		glColor3f(0.0f, 0.5f, 1.0f);

		glBegin(GL_TRIANGLES);

		glVertex3d(a.x, a.y, a.z);
		glVertex3d(b.x, b.y, b.z);
		glVertex3d(c.x, c.y, c.z);

		glEnd();

		glColor3f(0.0f, 0.0f, 0.0f);

		glBegin(GL_LINES);

		glVertex3d(a.x, a.y, a.z);
		glVertex3d(b.x, b.y, b.z);

		glVertex3d(b.x, b.y, b.z);
		glVertex3d(c.x, c.y, c.z);

		glVertex3d(a.x, a.y, a.z);
		glVertex3d(c.x, c.y, c.z);

		glEnd();


	}







	glDisable(GL_LIGHTING);

	// If we do draw the axis at all, make sure not to draw its outline.
	if(draw_axis && false == disable_colouring)
	{
		glEnable(GL_ALPHA);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glLineWidth(outline_width);

		glBegin(GL_LINES);

		glColor4f(0, 0, 0, 0.5);

		//		glColor4f(1, 0, 0, 0.25);
		glVertex3f(0, 0, 0);
		glVertex3f(1, 0, 0);
		//		glColor4f(0, 1, 0, 0.25);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 1, 0);
		//		glColor4f(0, 0, 1, 0.25);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 1);

		//glColor4f(0, 0, 0, 0.25);
		//glVertex3f(0, 0, 0);
		//glVertex3f(-1, 0, 0);
		//glVertex3f(0, 0, 0);
		//glVertex3f(0, -1, 0);
		//glVertex3f(0, 0, 0);
		//glVertex3f(0, 0, -1);

		glEnd();

		glDisable(GL_BLEND);
		glDisable(GL_ALPHA);
	}

	glPopMatrix();
}








#endif