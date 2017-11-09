#include <iostream> 
#include <fstream> 
#include <string>
#include "../../Detour/Include/DetourNavMesh.h"
#include <vector>
#include <sstream>
#include <unordered_map>
#include "../../Detour/Include/DetourNavMeshBuilder.h"
#include "../../Recast/Include/Recast.h"
#include <iosfwd>
#include "../../Recast/Include/RecastAlloc.h"

using namespace std;

float CELL_SIZE = 0.2266667f;		// 顶点精度。顶点坐标都是此数的整数倍
float TILE_SIZE = 58.026676f;
int TILE_SIZE_INT = TILE_SIZE / CELL_SIZE;
int NVP = 4;			// 每个poly最大顶点树

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

struct rcEdge
{
	unsigned short vert[2];
	unsigned short polyEdge[2];
	unsigned short poly[2];
};
bool buildMeshAdjacency(unsigned short* polys, const int npolys,
	const int nverts, const int vertsPerPoly)
{
	// Based on code by Eric Lengyel from:
	// http://www.terathon.com/code/edges.php

	int maxEdgeCount = npolys*vertsPerPoly;
	unsigned short* firstEdge = (unsigned short*)rcAlloc(sizeof(unsigned short)*(nverts + maxEdgeCount), RC_ALLOC_TEMP);
	if (!firstEdge)
		return false;
	unsigned short* nextEdge = firstEdge + nverts;
	int edgeCount = 0;

	rcEdge* edges = (rcEdge*)rcAlloc(sizeof(rcEdge)*maxEdgeCount, RC_ALLOC_TEMP);
	if (!edges)
	{
		rcFree(firstEdge);
		return false;
	}

	for (int i = 0; i < nverts; i++)
		firstEdge[i] = RC_MESH_NULL_IDX;

	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*vertsPerPoly*2];
		for (int j = 0; j < vertsPerPoly; ++j)
		{
			if (t[j] == RC_MESH_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= vertsPerPoly || t[j+1] == RC_MESH_NULL_IDX) ? t[0] : t[j+1];
			if (v0 < v1)
			{
				rcEdge& edge = edges[edgeCount];
				edge.vert[0] = v0;
				edge.vert[1] = v1;
				edge.poly[0] = (unsigned short)i;
				edge.polyEdge[0] = (unsigned short)j;
				edge.poly[1] = (unsigned short)i;
				edge.polyEdge[1] = 0;
				// Insert edge
				nextEdge[edgeCount] = firstEdge[v0];
				firstEdge[v0] = (unsigned short)edgeCount;
				edgeCount++;
			}
		}
	}

	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*vertsPerPoly*2];
		for (int j = 0; j < vertsPerPoly; ++j)
		{
			if (t[j] == RC_MESH_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= vertsPerPoly || t[j+1] == RC_MESH_NULL_IDX) ? t[0] : t[j+1];
			if (v0 > v1)
			{
				for (unsigned short e = firstEdge[v1]; e != RC_MESH_NULL_IDX; e = nextEdge[e])
				{
					rcEdge& edge = edges[e];
					if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1])
					{
						edge.poly[1] = (unsigned short)i;
						edge.polyEdge[1] = (unsigned short)j;
						break;
					}
				}
			}
		}
	}

	// Store adjacency
	for (int i = 0; i < edgeCount; ++i)
	{
		const rcEdge& e = edges[i];
		if (e.poly[0] != e.poly[1])
		{
			unsigned short* p0 = &polys[e.poly[0]*vertsPerPoly*2];
			unsigned short* p1 = &polys[e.poly[1]*vertsPerPoly*2];
			p0[vertsPerPoly + e.polyEdge[0]] = e.poly[1];
			p1[vertsPerPoly + e.polyEdge[1]] = e.poly[0];
		}
	}

	rcFree(firstEdge);
	rcFree(edges);

	return true;
}

void FindPortalEdges(unsigned short* polys, const int npolys, unsigned short* verts,
	const int nverts, const int nvp)
{
	// Find portal edges
	//if (mesh.borderSize > 0)
	{
		const int w = TILE_SIZE_INT;
		const int h = TILE_SIZE_INT;
		for (int i = 0; i < npolys; ++i)
		{
			unsigned short* p = &polys[i*2*nvp];
			for (int j = 0; j < nvp; ++j)
			{
				if (p[j] == RC_MESH_NULL_IDX) break;
				// Skip connected edges.
				if (p[nvp+j] != RC_MESH_NULL_IDX)
					continue;
				int nj = j+1;
				if (nj >= nvp || p[nj] == RC_MESH_NULL_IDX) nj = 0;
				const unsigned short* va = &verts[p[j]*3];
				const unsigned short* vb = &verts[p[nj]*3];

				if ((int)va[0] == 0 && (int)vb[0] == 0)
					p[nvp+j] = 0x8000 | 0;
				else if ((int)va[2] == h && (int)vb[2] == h)
					p[nvp+j] = 0x8000 | 1;
				else if ((int)va[0] == w && (int)vb[0] == w)
					p[nvp+j] = 0x8000 | 2;
				else if ((int)va[2] == 0 && (int)vb[2] == 0)
					p[nvp+j] = 0x8000 | 3;
			}
		}
	}
}

enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP,
};
enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK		= 0x01,		// Ability to walk (ground, grass, road)
	SAMPLE_POLYFLAGS_SWIM		= 0x02,		// Ability to swim (water).
	SAMPLE_POLYFLAGS_DOOR		= 0x04,		// Ability to move through doors.
	SAMPLE_POLYFLAGS_JUMP		= 0x08,		// Ability to jump.
	SAMPLE_POLYFLAGS_DISABLED	= 0x10,		// Disabled polygon
	SAMPLE_POLYFLAGS_ALL		= 0xffff	// All abilities.
};

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

struct XVector3
{
	XVector3(float _x, float _y, float _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}
	XVector3()
	{
		x = 0;
		y = 0;
		z = 0;
	}
	float x;
	float y;
	float z;

	XVector3 operator+ (const XVector3& other) const
	{
		XVector3 v;
		v.x = x + other.x;
		v.y = y + other.y;
		v.z = z + other.z;
		return v;
	}

	XVector3& operator+= (const XVector3& other)
	{
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}

	XVector3& operator*= (float r)
	{
		x *= r;
		y *= r;
		z *= r;
		return *this;
	}

	void CopyByCell(unsigned short* v) const
	{
		v[0] = x / CELL_SIZE;
		v[1] = y / CELL_SIZE;
		v[2] = z / CELL_SIZE;
	}
};

struct XPoly
{
	std::vector<int> pointList;
};

;
struct XTile
{
	union XTileIndex
	{
		int value;
		struct 
		{
			short x;
			short z;
		} pos;
	}index;

	std::vector<XPoly> polyList;
	unsigned short* pVerts;
	int nVerts;
	unsigned short* pPolys;
	unsigned short* pFlags;
	unsigned char* pAreas;
	int nPolys;
	float bmin[3];
	float bmax[3];

	static XTileIndex CalcIndex(const XVector3& center)
	{
		XTileIndex index;
		index.pos.x = center.x / TILE_SIZE;
		index.pos.z	= center.z / TILE_SIZE;
		return index;
	}

	void Generate(const std::vector<XVector3>& pointList)
	{
		bmin[0] = 0 + index.pos.x * TILE_SIZE;
		bmin[1] = 0;
		bmin[2] = 0 + index.pos.z * TILE_SIZE;

		bmax[0] = bmin[0] + TILE_SIZE;
		bmax[1] = 1000;
		bmax[2] = bmin[2] + TILE_SIZE;

		std::unordered_map<int, int> point2Vert;
		nVerts = 0;

		nPolys = polyList.size();
		pPolys = new unsigned short[nPolys * NVP * 2];
		pFlags = new unsigned short[nPolys];
		pAreas = new unsigned char[nPolys];
		memset(pPolys, 0xff, nPolys * NVP * 2 * sizeof(unsigned short));
		memset(pFlags, 0, nPolys * sizeof(unsigned short));
		memset(pAreas, 0, nPolys * sizeof(unsigned char));

		for (int i = 0; i < nPolys; ++i)
		{
			if (pAreas[i] == RC_WALKABLE_AREA)
				pAreas[i] = SAMPLE_POLYAREA_GROUND;

			if (pAreas[i] == SAMPLE_POLYAREA_GROUND ||
				pAreas[i] == SAMPLE_POLYAREA_GRASS ||
				pAreas[i] == SAMPLE_POLYAREA_ROAD)
			{
				pFlags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (pAreas[i] == SAMPLE_POLYAREA_WATER)
			{
				pFlags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
		}
		for (auto it = polyList.begin(); it != polyList.end(); ++it)
		{
			for (auto it2 = it->pointList.begin(); it2 != it->pointList.end(); ++it2)
			{
				int index = *it2;
				if (point2Vert.find(index) == point2Vert.end())
				{
					point2Vert[index] = nVerts++;
				}
			}
		}

		pVerts = new unsigned short[nVerts * 3];
		for (auto it = point2Vert.begin(); it != point2Vert.end(); ++it)
		{
			const XVector3 v(pointList[it->first] + XVector3(-bmin[0], -bmin[1], -bmin[2]));
			v.CopyByCell(pVerts + it->second * 3);
		}

		for (int i = 0; i < polyList.size(); ++i)
		{
			const XPoly& poly = polyList[i];
			for (int j = 0; j < poly.pointList.size(); ++j)
			{
				pPolys[i * NVP * 2 + j] = point2Vert[poly.pointList[j]];
			}
		}

		if (!buildMeshAdjacency(pPolys, nPolys, nVerts, NVP))
		{
			std::cout << "ERROR" << std::endl;
			return;
		}

		FindPortalEdges(pPolys, nPolys, pVerts, nVerts, NVP);

		std::cout << "Tile[" << index.pos.x << ", " << index.pos.z << "], pPolys: ";
		for (int i = 0; i < nPolys * NVP * 2; ++i)
		{
			std::cout << pPolys[i] << " ";
		}

		std::cout << std::endl;
	}
};

void SaveNavMesh(const char* path, const dtNavMesh* mesh)
{
	if (!mesh) return;

	FILE* fp = fopen(path, "wb");
	if (!fp)
		return;

	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}

void MakeNavData(const std::vector<XVector3>& pointList, std::unordered_map<long long, XTile>& tileList)
{
	// Set header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = tileList.size();
	
	header.params.tileWidth = TILE_SIZE;
	header.params.tileHeight = TILE_SIZE;
	header.params.orig[0] = 0.f;
	header.params.orig[1] = 0.f;
	header.params.orig[2] = 0.f;
	header.params.maxTiles = 1000;
	header.params.maxPolys = 1000;

	dtNavMesh* mesh = dtAllocNavMesh();
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
		return;

	for (auto it = tileList.begin(); it != tileList.end(); ++it)
	{
		XTile& tile = it->second;
		tile.Generate(pointList);

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = tile.pVerts;
		params.vertCount = tile.nVerts;
		params.polys = tile.pPolys;
		params.polyAreas = tile.pAreas;
		params.polyFlags = tile.pFlags;
		params.polyCount = tile.nPolys;
		params.nvp = NVP;
		params.detailMeshes = NULL;
		params.detailVerts = NULL;
		params.detailVertsCount = 0;
		params.detailTris = NULL;
		params.detailTriCount = 0;
		params.offMeshConVerts = NULL;
		params.offMeshConRad = NULL;
		params.offMeshConDir = NULL;
		params.offMeshConAreas = NULL;
		params.offMeshConFlags = NULL;
		params.offMeshConUserID = NULL;
		params.offMeshConCount = NULL;
		params.walkableHeight = 10.f;
		params.walkableRadius = 0.1f;
		params.walkableClimb = 0.2f;
		params.tileX = tile.index.pos.x;
		params.tileY = tile.index.pos.z;
		params.tileLayer = 0;
		rcVcopy(params.bmin, tile.bmin);
		rcVcopy(params.bmax, tile.bmax);
		params.cs = CELL_SIZE;
		params.ch = CELL_SIZE;
		params.buildBvTree = true;


		unsigned char* navData = 0;
		int navDataSize = 0;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{

			continue;
		}		

		dtStatus status = mesh->addTile(navData,navDataSize,DT_TILE_FREE_DATA,0,0);
		if (dtStatusFailed(status))
			dtFree(navData);

	}


	/// Output to File
	SaveNavMesh("all_tiles_navmesh.bin", mesh);
}

int main()
{
	ifstream fs;
	fs.open("Data/navinfo.txt");

	std::vector<XVector3> pointList;
	std::unordered_map<long long, XTile> tileList;

	char buffer[200];
	int mode = 0;
	while(fs)
	{
		fs.getline(buffer, 200);

		if (buffer[0] == 'v')
		{
			mode = 1;
			continue;
		}
		else if (buffer[0] == 'p')
		{
			mode = 2;
			continue;
		}
		//else if (buffer[0] == 'a')
		//{
		//	mode = 3;
		//	continue;
		//}
		else if (buffer[0] == 'n')	// nvp
		{
			mode = 4;
			fs.getline(buffer, 200);
			fs >> NVP;
			continue;
		}

		stringstream ss(buffer);
		if (mode == 1)
		{
			XVector3 vec;
			ss >> vec.x >> vec.y >> vec.z;
			pointList.push_back(vec);
		}
		else if (mode == 3)
		{

		}
		else if (mode == 2)
		{
			XPoly poly;
			int index;
			while ( ss >> index)
			{
				poly.pointList.push_back(index);
			}
			if (poly.pointList.size() == 0)
				continue;

			XVector3 pointTotal;
			for (auto it = poly.pointList.begin(); it != poly.pointList.end(); ++it)
			{
				pointTotal += pointList[*it];
			}
			pointTotal *= (1.0f/poly.pointList.size());

			XTile* tile;

			XTile::XTileIndex tileIndex = XTile::CalcIndex(pointTotal);
			auto it = tileList.find(tileIndex.value);
			if (it == tileList.end())
			{
				XTile newTile;
				newTile.index = tileIndex;
				tileList[tileIndex.value] = newTile;
				it = tileList.find(tileIndex.value);
			}

			tile = &(it->second);
			tile->polyList.push_back(poly);
		}
	}

	fs.close();

	MakeNavData(pointList, tileList);
	return 0;
}