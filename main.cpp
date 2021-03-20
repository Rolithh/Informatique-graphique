#include <iostream>
#include <algorithm>
#include <math.h>

#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <random>
static std::default_random_engine engine(10);
static std::uniform_real_distribution<double> uniform(0,1);

using namespace std;

#include <string>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <list>



// Vector
class Vector {
public:
    explicit Vector(double x=0, double y=0, double z=0){
        coords[0] = x;
        coords[1] = y;
        coords[2] = z;
    };

    double operator[](int i) const {return coords[i]; };
    double &operator[](int i) {return coords[i]; };
    Vector &operator+=(const Vector& v){
        coords[0] += v[0];
        coords[1] += v[1];
        coords[2] += v[2];
        return *this;
    }

    double square_norm() const {return coords[0]*coords[0] + coords[1]*coords[1] + coords[2]*coords[2]; };
    Vector get_normalized() const {
        double norm = sqrt(square_norm());
        return Vector(coords[0]/norm, coords[1]/norm, coords[2]/norm);
    };


private:
    double coords[3];
};

Vector operator+(const Vector& a, const Vector& b){
    return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}


Vector operator-(const Vector& a, const Vector& b){
    return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

Vector operator-(const Vector& v){
    return Vector(-v[0], -v[1], -v[2]);
}

Vector operator*(const Vector& a, const Vector& b){
    return Vector(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}

Vector operator*(const Vector& v, double lambda){
    return Vector(v[0]*lambda, v[1]*lambda, v[2]*lambda);
}

Vector operator*(double lambda, const Vector& v){
    return Vector(v[0]*lambda, v[1]*lambda, v[2]*lambda);
}

Vector operator/(const Vector& v, double lambda){
    return Vector(v[0]/lambda, v[1]/lambda, v[2]/lambda);
}

Vector cross(const Vector &a, const Vector &b){
    return Vector(a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]);
}

double dot(const Vector& a, const Vector& b){
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

double sqr(double a){
    return a*a;
}

// Ray
class Ray{

public:
    explicit Ray(Vector C, Vector u) : C(C), u(u){};

    Vector C;
    Vector u;

};

class BoundingBox{
public:
    BoundingBox(Vector P1=Vector(0,0,0), Vector P2=Vector(0,0,0)) : P1(P1), P2(P2) {};
    Vector P1, P2;

    bool intersect(const Ray& r){
        double t1x = (P1[0] - r.C[0])/r.u[0];
        double t2x = (P2[0] - r.C[0])/r.u[0];
        double txmin = std::min(t1x, t2x);
        double txmax = std::max(t1x, t2x);

        double t1y = (P1[1] - r.C[1])/r.u[1];
        double t2y = (P2[1] - r.C[1])/r.u[1];
        double tymin = std::min(t1y, t2y);
        double tymax = std::max(t1y, t2y);

        double t1z = (P1[2] - r.C[2])/r.u[2];
        double t2z = (P2[2] - r.C[2])/r.u[2];
        double tzmin = std::min(t1z, t2z);
        double tzmax = std::max(t1z, t2z);

        double tmin = std::max(txmin, std::max(tymin, tzmin));
        double tmax = std::min(txmax, std::min(tymax, tzmax));

        return tmax > 0 && tmin < tmax;
    }
};

class Node{
public:
    Node *left_child, *right_child;
    BoundingBox bb;
    int start_index, end_index;
};

class Object{
public:
    Object() {};
    virtual bool intersect(const Ray& r, Vector& P, Vector& n, double &t, Vector &color) = 0;

    Vector albedo;
    bool mirror, transparent;
};


// Sphere
class Sphere: public Object{

public:
    Sphere(Vector O, double R, Vector albedo=Vector(1,1,1), bool mirror=false, bool transparent=false) : O(O), R(R)
    {
        this->albedo = albedo;
        this->mirror = mirror;
        this->transparent = transparent;
    };

    Vector O;
    double R;




    bool intersect(const Ray& r, Vector& P, Vector& n, double& t, Vector &color){
        color = this->albedo;
        Vector u = r.u.get_normalized();
        double b = 2*dot(u, r.C - O);
        double c = (r.C - O).square_norm() - R*R;
        double delta = b*b - 4*c;
        if(delta < 0) return false;
        double sqrtDelta = sqrt(delta);

        double t2 = (-b + sqrtDelta)/2;
        if (t2 < 0) return false;

        double t1 = (-b - sqrtDelta)/2;
        if (t1 > 0){
            t = t1;
            P = r.C + t1*u;
        } else {
            t = t2;
            P = r.C + t2*u;
        }

        n = (P - O).get_normalized();
        return true;
    };
};



Vector random_cos(const Vector &n){
    double r1 = uniform(engine), r2 = uniform(engine);
    double x = cos(2*M_PI*r1)*sqrt(1 - r2);
    double y = sin(2*M_PI*r1)*sqrt(1 - r2);
    double z = sqrt(r2);
    Vector T1;
    if(n[0] <= n[1] && n[0] <= n[2]){
        T1 = Vector(0, -n[2], n[1]);
    } else if(n[1] <= n[0] && n[1] <= n[2]){
        T1 = Vector(-n[2],0,n[0]);
    } else {
        T1 = Vector(-n[1],n[0],0);
    }
    T1 = T1.get_normalized();
    Vector T2 = cross(n, T1);
    return z*n + x*T1 + y*T2;
}


class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
	};
	int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
	int uvi, uvj, uvk;  // indices within the uv coordinates array
	int ni, nj, nk;  // indices within the normals array
	int group;       // face group
};


class TriangleMesh : public Object {
public:
    ~TriangleMesh() {}
	TriangleMesh() {
        this->albedo = Vector(1,1,1);
        this->transparent = false;
        this->mirror = false;
        this->bbtree = new Node;
	};
    std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
	std::vector<unsigned char*> textures;
	std::vector<int> w_texture, h_texture;
	Node *bbtree;



	BoundingBox buildBB(int start_index, int end_index){
	    BoundingBox bb;
        bb.P1 = Vector(1E9,1E9,1E9);
        bb.P2 = Vector(-1E9,-1E9,-1E9);

        for(int i=start_index; i < end_index; i++){
            for(int j=0; j <3; j++){
                bb.P1[j] = std::min(bb.P1[j], vertices[indices[i].vtxi][j]);
                bb.P2[j] = std::max(bb.P2[j], vertices[indices[i].vtxi][j]);
                bb.P1[j] = std::min(bb.P1[j], vertices[indices[i].vtxj][j]);
                bb.P2[j] = std::max(bb.P2[j], vertices[indices[i].vtxj][j]);
                bb.P1[j] = std::min(bb.P1[j], vertices[indices[i].vtxk][j]);
                bb.P2[j] = std::max(bb.P2[j], vertices[indices[i].vtxk][j]);
            }
        }
        return bb;
	};

    void buildBVH(Node* n, int start_index, int end_index){
        n->start_index = start_index;
        n->end_index = end_index;
        n->bb = buildBB(n->start_index, n->end_index);

        Vector diag = n->bb.P2 - n->bb.P1;
        int dim;
        if(diag[0] >= diag[1] && diag[0] >= diag[2]){
            dim = 0;
        }else{
            if(diag[1] >= diag[0] && diag[1] >= diag[2]){
                dim = 1;
            }else{
                dim = 2;
            }
        }
        double middle = (n->bb.P1[dim] + n->bb.P2[dim])*0.5;
        int pivot = n->start_index;
        for(int i = n->start_index; i < n->end_index; i++){
            double triangle_middle = (vertices[indices[i].vtxi][dim] + vertices[indices[i].vtxj][dim] + vertices[indices[i].vtxk][dim])/3;
            if(triangle_middle < middle){
                std::swap(indices[i], indices[pivot]);
                pivot++;
            }
        }
        n->left_child = nullptr;
        n->right_child = nullptr;
        if(pivot == start_index || pivot == end_index || (end_index-start_index) < 5) return;

        n->left_child = new Node;
        n->right_child = new Node;

        buildBVH(n->left_child, n->start_index, pivot);
        buildBVH(n->right_child, pivot, n->end_index);
	}

	void loadTexture(const char* path){
	    int W, H, C;
        unsigned char* texture = stbi_load(path, &W, &H, &C, 3);
        w_texture.push_back(W);
        h_texture.push_back(H);

        textures.push_back(texture);
	}


	void readOBJ(const char* obj) {

		char matfile[255];
		char grp[255];

		FILE* f;
		f = fopen(obj, "r");
		int curGroup = -1;
		while (!feof(f)) {
			char line[255];
			if (!fgets(line, 255, f)) break;

			std::string linetrim(line);
			linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
			strcpy(line, linetrim.c_str());

			if (line[0] == 'u' && line[1] == 's') {
				sscanf(line, "usemtl %[^\n]\n", grp);
				curGroup++;
			}

			if (line[0] == 'v' && line[1] == ' ') {
				Vector vec;

				Vector col;
				if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
					col[0] = std::min(1., std::max(0., col[0]));
					col[1] = std::min(1., std::max(0., col[1]));
					col[2] = std::min(1., std::max(0., col[2]));

					vertices.push_back(vec);
					vertexcolors.push_back(col);

				} else {
					sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
					vertices.push_back(vec);
				}
			}
			if (line[0] == 'v' && line[1] == 'n') {
				Vector vec;
				sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
				normals.push_back(vec);
			}
			if (line[0] == 'v' && line[1] == 't') {
				Vector vec;
				sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
				uvs.push_back(vec);
			}
			if (line[0] == 'f') {
				TriangleIndices t;
				int i0, i1, i2, i3;
				int j0, j1, j2, j3;
				int k0, k1, k2, k3;
				int nn;
				t.group = curGroup;

				char* consumedline = line + 1;
				int offset;

				nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
				if (nn == 9) {
					if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
					if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
					if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
					if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
					if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
					if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
					if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
					if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
					if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
					indices.push_back(t);
				} else {
					nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
					if (nn == 6) {
						if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
						if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
						if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
						if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
						if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
						if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
						indices.push_back(t);
					} else {
						nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
						if (nn == 3) {
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							indices.push_back(t);
						} else {
							nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
							if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
							if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
							indices.push_back(t);
						}
					}
				}

				consumedline = consumedline + offset;

				while (true) {
					if (consumedline[0] == '\n') break;
					if (consumedline[0] == '\0') break;
					nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
					TriangleIndices t2;
					t2.group = curGroup;
					if (nn == 3) {
						if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
						if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
						if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
						if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
						if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
						if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
						if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
						if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
						if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
						indices.push_back(t2);
						consumedline = consumedline + offset;
						i2 = i3;
						j2 = j3;
						k2 = k3;
					} else {
						nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
						if (nn == 2) {
							if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
							if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
							if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
							if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
							if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
							if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
							consumedline = consumedline + offset;
							i2 = i3;
							j2 = j3;
							indices.push_back(t2);
						} else {
							nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
							if (nn == 2) {
								if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
								if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
								if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
								if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
								if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
								if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
								consumedline = consumedline + offset;
								i2 = i3;
								k2 = k3;
								indices.push_back(t2);
							} else {
								nn = sscanf(consumedline, "%u%n", &i3, &offset);
								if (nn == 1) {
									if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
									if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
									if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
									consumedline = consumedline + offset;
									i2 = i3;
									indices.push_back(t2);
								} else {
									consumedline = consumedline + 1;
								}
							}
						}
					}
				}

			}

		}
		fclose(f);

	}

	bool intersect(const Ray& r, Vector& P, Vector& n, double& t, Vector &color){


	    if(!bbtree->bb.intersect(r)) return false;

	    std::list<Node*> l;
	    l.push_back(bbtree);
        t = 1E9;
        bool has_inter = false;
	    while(!l.empty()){
            Node* c = l.front();
            l.pop_front();
            if(c->left_child && c->right_child){
                if(c->left_child->bb.intersect(r)){
                    l.push_front(c->left_child);
                }
                if(c->right_child->bb.intersect(r)){
                    l.push_front(c->right_child);
                }

            }else{
                for(int i=c->start_index; i < c->end_index; i++){
                    const Vector &A = vertices[indices[i].vtxi];
                    const Vector &B = vertices[indices[i].vtxj];
                    const Vector &C = vertices[indices[i].vtxk];

                    Vector e1 = B - A;
                    Vector e2 = C - A;
                    Vector N = cross(e1, e2);
                    Vector AO = r.C - A;
                    Vector AOu = cross(AO, r.u);
                    double invUN = 1./dot(r.u, N);
                    double beta = -dot(e2, AOu)*invUN;
                    double gamma = dot(e1, AOu)*invUN;
                    double alpha = 1 - beta - gamma;
                    double localt = -dot(AO, N)*invUN;

                    if(beta >= 0 && gamma >= 0 && beta <= 1 && gamma <= 1 && alpha >= 0 && localt >= 0){
                        has_inter = true;
                        if (localt < t){
                            t = localt;
                            //n = N.get_normalized();
                            n = alpha*normals[indices[i].ni] + beta*normals[indices[i].nj] + gamma*normals[indices[i].nk];
                            n = n.get_normalized();
                            P = r.C + t*r.u;

                            int W = w_texture[indices[i].group];
                            int H = h_texture[indices[i].group];
                            Vector UV = alpha*uvs[indices[i].uvi] + beta*uvs[indices[i].uvj] + gamma*uvs[indices[i].uvk];
                            UV = UV*Vector(W, H, 0);
                            int uvx = UV[0] + 0.5;
                            int uvy = UV[1] + 0.5;
                            uvx = uvx % W;
                            uvy = uvy % H;
                            if(uvx < 0) uvx += W;
                            if(uvy < 0) uvy += H;
                            uvy = H - uvy -1;
                            color = Vector(textures[indices[i].group][(uvy*W + uvx)*3], textures[indices[i].group][(uvy*W + uvx)*3 +1], textures[indices[i].group][(uvy*W + uvx)*3 +2])/255;
                            //color = Vector(std::pow(textures[indices[i].group][(uvy*W + uvx)*3],2.2), std::pow(textures[indices[i].group][(uvy*W + uvx)*3 +1],2.2), std::pow(textures[indices[i].group][(uvy*W + uvx)*3 +2],2.2))/255;
                            color = Vector(std::pow(color[0], 2.2), std::pow(color[1], 2.2), std::pow(color[2], 2.2));
                        }
                    }
                }
            }
	    }
        return has_inter;
	};
};



// Scene
class Scene {
public:
//
    Scene(){objects = std::vector<Object*>(); };

    std::vector<Object*> objects;
    Vector L = Vector(-10,20,40);
    Vector I = Vector(1,1,1)*5E9;

    bool intersect(const Ray& r, Vector& P, Vector& n, Vector& albedo, bool& mirror, bool& transparent, double& d, int& objectid){
        bool has_inter = false;
        double t = 1E9;
        for(int i = 0; i < objects.size(); i++){
            Vector localP, localn, color;
            double localt;
            if (objects[i]->intersect(r, localP, localn, localt, color) && localt <= t){
                has_inter = true;
                t = localt;
                d = (r.C - localP).square_norm();
                n = localn;
                P = localP;

                albedo = color;
                mirror = objects[i]->mirror;
                transparent = objects[i]->transparent;

                objectid = i;

            }


        }
        return has_inter;
    };


    Vector getColor(Ray r, int bounce=0, bool lastdiffuse=false){
        Sphere* light = dynamic_cast<Sphere*>(objects[0]);
        Vector color(0,0,0);
        if (bounce > 5){
            return color;
        }
        Vector P, n, albedo;
        double interd;
        int objectid;
        bool mirror, transparent;
        if(intersect(r, P, n, albedo, mirror, transparent, interd, objectid)){

            if(objectid == 0){
                if(bounce == 0 || !lastdiffuse){
                    double R = light->R;
                    return I/(4*M_PI*M_PI*R*R);
                }else{
                    return Vector(0,0,0);
                }
            }

            if(mirror){
                Vector reflectedDir = r.u - 2*dot(r.u, n)*n;
                Ray reflectedRay(P + 1E-5*n, reflectedDir);
                return getColor(reflectedRay, bounce + 1);
            }
            if(transparent){

                double n1 = 1, n2 = 1.4;
                Vector n_ = n;
                if(dot(r.u,n) > 0){
                    std::swap(n1,n2);
                    n_ = -n;
                }
                Vector Tt = n1/n2*(r.u - dot(r.u,n_)*n_);
                double rad = 1 - sqr(n1/n2)*(1 - sqr(dot(r.u, n_)));
                if(rad < 0){
                    Vector reflectedDir = r.u - 2*dot(r.u, n_)*n_;
                    Ray reflectedRay(P + 1E-5*n_, reflectedDir);
                    return getColor(reflectedRay, bounce + 1);
                }
                Vector Tn = -sqrt(rad)*n_;
                Vector refractedDir = Tt + Tn;
                Ray refractedRay(P - 1E-5*n_, refractedDir);
                return getColor(refractedRay, bounce);
            }
//            Vector PL = L - P;
//            double d = sqrt(PL.square_norm());
//            Ray shadowRay = Ray(P + 1E-5*n, PL/d);
//            Vector shadowP, shadown, shadowAlbedo;
//            double shadowd;
//            bool mirror2, transp2;
//            bool shadowInter = intersect(shadowRay, shadowP, shadown, shadowAlbedo, mirror2, transp2, shadowd);
//            if (shadowInter && shadowd < d*d){
//                color = Vector(0,0,0);
//            } else {
//                color = (I/(4*M_PI*d*d)*albedo/M_PI*std::max(0., dot(n, PL/d)));
//            }

            // Eclairage direct
            Vector PL = L - P;
            PL = PL.get_normalized();
            Vector w = random_cos(-PL);
            Vector xprime = w*light->R + light->O;
            Vector Pxprime = xprime - P;
            double d = sqrt(Pxprime.square_norm());
            Pxprime = Pxprime/d;

            Vector shadowP, shadown, shadowAlbedo;
            double shadowd;
            bool mirror2, transp2;
            int objectid;
            Ray shadowRay(P + 1e-5*n, Pxprime);
            bool shadowInter = intersect(shadowRay, shadowP, shadown, shadowAlbedo, mirror2, transp2, shadowd, objectid);
            if (shadowInter && shadowd < sqr(d - 1e-5) && objectid != 0){
                color = Vector(0,0,0);
            } else {
                double proba = std::max(1e-8, dot(-PL, w))/(M_PI*sqr(light->R));
                double J = std::max(1e-8, dot(w, -Pxprime))/(d*d);
                color = I/(4*sqr(M_PI*light->R))*albedo/M_PI*std::max(0., dot(n, Pxprime))*J/proba;
            }

            // Eclairage indirect
            Vector wi = random_cos(n);

            Ray wiRay(P + 1E-5*n, wi);

            color += albedo*getColor(wiRay, bounce + 1, true);

        }
        return color;
    }
};



//double monte_carlo_1D(){
//    double sigma = 0.25;
//    double C = 1/(sigma*sqrt(2*M_PI));
//    int N = 10000;
//
//    double S = 0;
//    for(int k = 0; k < N; k++){
//        double r1 = uniform(engine), r2 = uniform(engine);
//        double xi = cos(2*M_PI*r1)*sqrt(-2*log(r2))*sigma;
//
//        if(xi < M_PI/2 && xi > -M_PI/2){
//            double f_xi = pow(cos(xi), 10);
//            double p_xi = C*exp(-sqr(xi/sigma)/2);
//            S += f_xi/p_xi/N;
//        }
//    }
//    cout << S;
//}
//
//double monte_carlo_4D(){
//    double sigma = 1;
//    double C = 1/(pow(sigma,4)*sqr(2*M_PI));
//    int N = 1000000;
//
//    double S = 0;
//    for(int k = 0; k < N; k++){
//        double r1 = uniform(engine), r2 = uniform(engine);
//        double r3 = uniform(engine), r4 = uniform(engine);
//        double xi = cos(2*M_PI*r1)*sqrt(-2*log(r2))*sigma;
//        double yi = sin(2*M_PI*r1)*sqrt(-2*log(r2))*sigma;
//        double zi = cos(2*M_PI*r3)*sqrt(-2*log(r4))*sigma;
//        double wi = sin(2*M_PI*r3)*sqrt(-2*log(r4))*sigma;
//
//        if(xi < M_PI/2 && xi > -M_PI/2 && yi < M_PI/2 && yi > -M_PI/2 && zi < M_PI/2 && zi > -M_PI/2 && wi < M_PI/2 && wi > -M_PI/2){
//            double f = sqr(cos(xi + yi + zi + wi));
//            double p = C*exp(-(sqr(xi/sigma) + sqr(yi/sigma) + sqr(zi/sigma) + sqr(wi/sigma))/2);
//            S += f/p/N;
//        }
//
//
//
//    }
//
//    cout << S;
//}


int main()
{
    int W = 512;
	int H = 512;

    Scene scene;

    Sphere SLight = Sphere(Vector(-10,20,40), 5, Vector(0,1,1));
    Sphere S = Sphere(Vector(0,0,0), 10, Vector(1,1,1), false, false);
    Sphere S1 = Sphere(Vector(0,1000,0), 940, Vector(1,0,0));
    Sphere S2 = Sphere(Vector(0,0,-1000), 940, Vector(0,1,0));
    Sphere S3 = Sphere(Vector(0,-1000,0), 990, Vector(0,0,1));
    Sphere S4 = Sphere(Vector(0,0,1000), 940, Vector(1,0,1));
    Sphere S5 = Sphere(Vector(1000,0,0), 940, Vector(1,1,0));
    Sphere S6 = Sphere(Vector(-1000,0,0), 940, Vector(0,1,1));

    TriangleMesh mesh;
    mesh.readOBJ("dog.obj");

    for(int i = 0; i < mesh.vertices.size(); i++){
        mesh.vertices[i][1] += 22;
        std::swap(mesh.vertices[i][1], mesh.vertices[i][2]);
        mesh.vertices[i][1] -= 10;
        mesh.vertices[i][2] = -mesh.vertices[i][2] +25;
    }
    for(int i = 0; i < mesh.normals.size(); i++){
        std::swap(mesh.normals[i][1], mesh.normals[i][2]);
        mesh.normals[i][2] = -mesh.normals[i][2];
    }
    //mesh.buildBB();
    mesh.buildBVH(mesh.bbtree, 0, mesh.indices.size());
    mesh.loadTexture("Australian_Cattle_Dog_dif.jpg");

    scene.objects.push_back(&SLight);

    //scene.objects.push_back(&S);
	scene.objects.push_back(&S1);
	scene.objects.push_back(&S2);
	scene.objects.push_back(&S3);
	scene.objects.push_back(&S4);
	scene.objects.push_back(&S5);
	scene.objects.push_back(&S6);

	scene.objects.push_back(&mesh);

	Vector C = Vector(20,30,55);

    int nrays = 50;

    double alpha = -30*M_PI/180;
    double beta = -30*M_PI/180;

    Vector up(0, cos(alpha), sin(alpha));
    //Vector up(0,1,0);
    Vector right(cos(beta), 0, sin(beta));

    double up0 = up[0];
    up[0] = cos(beta)*up0 - sin(beta)*up[2];
    up[2] = sin(beta)*up0 + cos(beta)*up[2];

    Vector viewDir = cross(right, up);

    double depth = -double(W)/2/tan(M_PI/6);

	std::vector<unsigned char> image(W*H * 3, 0);
#pragma omp paralellel for schedule(dynamic, 1)
	for (int i = 0; i < H; i++) {
        cout << i << "\n";
		for (int j = 0; j < W; j++) {


            Vector color(0,0,0);


            for(int k = 0; k < nrays; k++){

                double r1 = uniform(engine), r2 = uniform(engine);
                double x = 0.25*cos(2*M_PI*r1)*sqrt(-2*log(r2));
                double y = 0.25*sin(2*M_PI*r1)*sqrt(-2*log(r2));

                r1 = uniform(engine);
                r2 = uniform(engine);
                double x2 = 0.5*cos(2*M_PI*r1)*sqrt(-2*log(r2));
                double y2 = 0.5*sin(2*M_PI*r1)*sqrt(-2*log(r2));


                Vector u = Vector(j - W/2 + x - 0.5, i - H/2 + y - 0.5, depth);
                u = u[0]*right + u[1]*up + u[2]*viewDir;
                u = u.get_normalized();

                Vector target = C + 55*u;
                Vector Cprime = C + Vector(x2, y2, 0);
                Vector uprime = (target - Cprime).get_normalized();


                Ray r = Ray(Cprime, uprime);

                color += scene.getColor(r);
            }

            color = color/nrays;

			image[((H-i-1)*W + j) * 3 + 0] = std::min(255., std::pow(color[0],0.45));
			image[((H-i-1)*W + j) * 3 + 1] = std::min(255., std::pow(color[1],0.45));
			image[((H-i-1)*W + j) * 3 + 2] = std::min(255., std::pow(color[2],0.45));
		}
	}

	stbi_write_png("image.png", W, H, 3, &image[0], 0);
	return 0;

}
