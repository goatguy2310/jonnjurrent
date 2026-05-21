#pragma once

#include <map>
#include <string>
#include <fstream>

#include "../geometry/triangle_mesh.h"

template <typename AccelPolicy>
inline void readOBJ(std::string obj, TriangleMesh<AccelPolicy>& mesh) {
	std::ifstream f(obj);
	if (!f) return;

	std::vector<Vector>& vertices = mesh.vertices;
	std::vector<Vector>& normals = mesh.normals;
	std::vector<Vector>& uvs = mesh.uvs;
	std::vector<Vector>& vertexcolors = mesh.vertexcolors;
	std::vector<TriangleIndices>& indices = mesh.indices;

	std::map<std::string, int> mtls;
	int curGroup = -1, maxGroup = -1;

	// OBJ indices are 1-based and can be negative (relative), this normalizes them
	auto resolveIdx = [](int i, int size) {
		return i < 0 ? size + i : i - 1;
	};

	auto setFaceVerts = [&](TriangleIndices& t, int i0, int i1, int i2) {
		t.vtx[0] = resolveIdx(i0, vertices.size());
		t.vtx[1] = resolveIdx(i1, vertices.size());
		t.vtx[2] = resolveIdx(i2, vertices.size());
	};
	auto setFaceUVs = [&](TriangleIndices& t, int j0, int j1, int j2) {
		t.uv[0] = resolveIdx(j0, uvs.size());
		t.uv[1] = resolveIdx(j1, uvs.size());
		t.uv[2] = resolveIdx(j2, uvs.size());
	};
	auto setFaceNormals = [&](TriangleIndices& t, int k0, int k1, int k2) {
		t.n[0] = resolveIdx(k0, normals.size());
		t.n[1] = resolveIdx(k1, normals.size());
		t.n[2] = resolveIdx(k2, normals.size());
	};

	std::string line;
	while (std::getline(f, line)) {
		// Trim trailing whitespace
		line.erase(line.find_last_not_of(" \r\t\n") + 1);
		if (line.empty()) continue;

		const char* s = line.c_str();

		if (line.rfind("usemtl ", 0) == 0) {
			std::string matname = line.substr(7);
			auto result = mtls.emplace(matname, maxGroup + 1);
			if (result.second) {
				curGroup = ++maxGroup;
			} else {
				curGroup = result.first->second;
			}
		} else if (line.rfind("vn ", 0) == 0) {
			Vector v;
			sscanf(s, "vn %lf %lf %lf", &v[0], &v[1], &v[2]);
			normals.push_back(v);
		} else if (line.rfind("vt ", 0) == 0) {
			Vector v;
			sscanf(s, "vt %lf %lf", &v[0], &v[1]);
			uvs.push_back(v);
		} else if (line.rfind("v ", 0) == 0) {
			Vector pos, col;
			if (sscanf(s, "v %lf %lf %lf %lf %lf %lf", &pos[0], &pos[1], &pos[2], &col[0], &col[1], &col[2]) == 6) {
				for (int i = 0; i < 3; i++) col[i] = std::min(1.0, std::max(0.0, col[i]));
				vertexcolors.push_back(col);
			} else {
				sscanf(s, "v %lf %lf %lf", &pos[0], &pos[1], &pos[2]);
			}
			vertices.push_back(pos);
		}
		else if (line[0] == 'f') {
			int i[4], j[4], k[4], offset, nn;
			const char* cur = s + 1;
			TriangleIndices t;
			t.group = curGroup;

			// Try each face format: v/vt/vn, v/vt, v//vn, v
			if ((nn = sscanf(cur, "%d/%d/%d %d/%d/%d %d/%d/%d%n", &i[0], &j[0], &k[0], &i[1], &j[1], &k[1], &i[2], &j[2], &k[2], &offset)) == 9) {
				setFaceVerts(t, i[0], i[1], i[2]); 
				setFaceUVs(t, j[0], j[1], j[2]); 
				setFaceNormals(t, k[0], k[1], k[2]);
			} else if ((nn = sscanf(cur, "%d/%d %d/%d %d/%d%n", &i[0], &j[0], &i[1], &j[1], &i[2], &j[2], &offset)) == 6) {
				setFaceVerts(t, i[0], i[1], i[2]); 
				setFaceUVs(t, j[0], j[1], j[2]);
			} else if ((nn = sscanf(cur, "%d//%d %d//%d %d//%d%n", &i[0], &k[0], &i[1], &k[1], &i[2], &k[2], &offset)) == 6) {
				setFaceVerts(t, i[0], i[1], i[2]); 
				setFaceNormals(t, k[0], k[1], k[2]);
			} else if ((nn = sscanf(cur, "%d %d %d%n", &i[0], &i[1], &i[2], &offset)) == 3) {
				setFaceVerts(t, i[0], i[1], i[2]);
			}
			else continue;

			indices.push_back(t);
			cur += offset;

			// Fan triangulation for polygon faces (4+ vertices)
			while (*cur && *cur != '\n') {
				TriangleIndices t2;
				t2.group = curGroup;
				if ((nn = sscanf(cur, " %d/%d/%d%n", &i[3], &j[3], &k[3], &offset)) == 3) {
					setFaceVerts(t2, i[0], i[2], i[3]); 
					setFaceUVs(t2, j[0], j[2], j[3]); 
					setFaceNormals(t2, k[0], k[2], k[3]);
				} else if ((nn = sscanf(cur, " %d/%d%n", &i[3], &j[3], &offset)) == 2) {
					setFaceVerts(t2, i[0], i[2], i[3]); 
					setFaceUVs(t2, j[0], j[2], j[3]);
				} else if ((nn = sscanf(cur, " %d//%d%n", &i[3], &k[3], &offset)) == 2) {
					setFaceVerts(t2, i[0], i[2], i[3]); 
					setFaceNormals(t2, k[0], k[2], k[3]);
				} else if ((nn = sscanf(cur, " %d%n", &i[3], &offset)) == 1) {
					setFaceVerts(t2, i[0], i[2], i[3]);
				} else { 
					cur++; 
					continue; 
				}

				indices.push_back(t2);
				cur += offset;
				i[2] = i[3]; j[2] = j[3]; k[2] = k[3];
			}
		}
	}
	mesh.calcTriangleData();
}
