#include "../QuickHull.hpp"
#include "../MathUtils.hpp"
#include <iostream>
#include <random>
#include <cassert>

namespace quickhull {
	
	namespace tests {
		
		using FloatType = float;
		using vec3 = Vector3<FloatType>;
		
		// Setup RNG
		static std::mt19937 rng;
		static std::uniform_real_distribution<> dist(0,1);
		
		FloatType rnd(FloatType from, FloatType to) {
			return from + (FloatType)dist(rng)*(to-from);
		};
		
		void assertSameValue(FloatType a, FloatType b) {
			assert(std::abs(a-b)<0.0001f);
		}
		
		void testVector3() {
			typedef Vector3<FloatType> vec3;
			vec3 a(1,0,0);
			vec3 b(1,0,0);
			
			vec3 c = a.projection(b);
			assert( (c-a).getLength()<0.00001f);
			
			a = vec3(1,1,0);
			b = vec3(1,3,0);
			c = b.projection(a);
			assert( (c-vec3(2,2,0)).getLength()<0.00001f);
		}
		
		template <typename T>
		std::vector<Vector3<T>> createSphere(T radius, size_t M, Vector3<T> offset = Vector3<T>(0,0,0)) {
			std::vector<Vector3<T>> pc;
			const T pi = 3.14159f;
			for (int i=0;i<=M;i++) {
				FloatType y = std::sin(pi/2 + (FloatType)i/(M)*pi);
				FloatType r = std::cos(pi/2 + (FloatType)i/(M)*pi);
				FloatType K = FloatType(1)-std::abs((FloatType)((FloatType)i-M/2.0f))/(FloatType)(M/2.0f);
				const size_t pcount = (size_t)(1 + K*M + FloatType(1)/2);
				for (size_t j=0;j<pcount;j++) {
					FloatType x = pcount>1 ? r*std::cos((FloatType)j/pcount*pi*2) : 0;
					FloatType z = pcount>1 ? r*std::sin((FloatType)j/pcount*pi*2) : 0;
					pc.emplace_back(x+offset.x,y+offset.y,z+offset.z);
				}
			}
			return pc;
		}
		
		void sphereTest() {
			QuickHull<FloatType> qh;
			FloatType y = 1;
			for (;;) {
				auto pc = createSphere<FloatType>(1, 100, Vector3<FloatType>(0,y,0));
				auto hull = qh.getConvexHull(pc,true,false);
				y *= 15;
				y /= 10;
				if (hull.getVertexBuffer().size()==4) {
					break;
				}
			}
			
			// Test worst case scenario: more and more points on the unit sphere. All points should be part of the convex hull, as long as we can make epsilon smaller without
			// running out of numerical accuracy.
			size_t i =  1;
			FloatType eps = 0.002f;
			for (;;) {
				auto pc = createSphere<FloatType>(1, i, Vector3<FloatType>(0,0,0));
				auto hull = qh.getConvexHull(pc,true,false,eps);
				std::cout << i << ":" << pc.size() << " : " << hull.getVertexBuffer().size() << " at eps=" << eps << std::endl;
				if (qh.getDiagnostics().m_failedHorizonEdges) {
					// This should not happen
					assert(false);
					break;
				}
				if (pc.size() == hull.getVertexBuffer().size()) {
					// Fine, all the points on unit sphere do belong to the convex mesh.
					i += 1;
				}
				else {
					eps *= 0.5f;
					std::cout << "Epsilon to " << eps << std::endl;
				}
				
				if (i == 500) {
					break;
				}
			}
		}
		
		void testPlanarCase() {
			QuickHull<FloatType> qh;
			std::vector<vec3> pc;
			pc.emplace_back(-3.000000f, -0.250000f, -0.800000f);
			pc.emplace_back(-3.000000f, 0.250000f, -0.800000f);
			pc.emplace_back(-3.125000f, 0.250000f, -0.750000);
			pc.emplace_back(-3.125000f, -0.250000f, -0.750000);
			auto hull = qh.getConvexHull(pc,true,false);
			assert(hull.getIndexBuffer().size()==12);
			assert(hull.getVertexBuffer().size()==4);
		}
		
		void testHalfEdgeOutput() {
			QuickHull<FloatType> qh;
			
			// 8 corner vertices of a cube + tons of vertices inside. Output should be a half edge mesh with 12 faces (6 cube faces with 2 triangles per face) and 36 half edges (3 half edges per face).
			std::vector<vec3> pc;
			for (int h=0;h<1000;h++) {
				pc.emplace_back(rnd(-1,1),rnd(-1,1),rnd(-1,1));
			}
			for (int h=0;h<8;h++) {
				pc.emplace_back(h&1?-2:2, h&2?-2:2, h&4?-2:2);
			}
			HalfEdgeMesh<FloatType, size_t> mesh = qh.getConvexHullAsMesh(&pc[0].x, pc.size(), true);
			assert(mesh.m_faces.size() == 12);
			assert(mesh.m_halfEdges.size() == 36);
			assert(mesh.m_vertices.size() == 8);
		}
		
		void testPlanes() {
			Vector3<FloatType> N(1,0,0);
			Vector3<FloatType> p(2,0,0);
			Plane<FloatType> P(N,p);
			auto dist = mathutils::getSignedDistanceToPlane(Vector3<FloatType>(3,0,0), P);
			assertSameValue(dist,1);
			dist = mathutils::getSignedDistanceToPlane(Vector3<FloatType>(1,0,0), P);
			assertSameValue(dist,-1);
			dist = mathutils::getSignedDistanceToPlane(Vector3<FloatType>(1,0,0), P);
			assertSameValue(dist,-1);
			N = Vector3<FloatType>(2,0,0);
			P = Plane<FloatType>(N,p);
			dist = mathutils::getSignedDistanceToPlane(Vector3<FloatType>(6,0,0), P);
			assertSameValue(dist,8);
		}
		
		void run() {
			// Setup test env
			const size_t N = 200;
			std::vector<vec3> pc;
			QuickHull<FloatType> qh;
			ConvexHull<FloatType> hull;
			
			// Seed RNG using Unix time
			std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
			auto seed = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
			rng.seed((unsigned int)seed);
			
			// Test 1 : Put N points inside unit cube. Result mesh must have exactly 8 vertices because the convex hull is the unit cube.
			pc.clear();
			for (int i=0;i<8;i++) {
				pc.emplace_back(i&1 ? -1 : 1,i&2 ? -1 : 1,i&4 ? -1 : 1);
			}
			for (size_t i=0;i<N;i++)
			{
				pc.emplace_back(rnd(-1,1),rnd(-1,1),rnd(-1,1));
			}
			hull = qh.getConvexHull(pc,true,false);
			assert(hull.getVertexBuffer().size()==8);
			assert(hull.getIndexBuffer().size()==3*2*6); // 6 cube faces, 2 triangles per face, 3 indices per triangle
			assert(&(hull.getVertexBuffer()[0])!=&(pc[0]));
			auto hull2 = hull;
			assert(hull2.getVertexBuffer().size()==hull.getVertexBuffer().size());
			assert(hull2.getVertexBuffer()[0].x==hull.getVertexBuffer()[0].x);
			assert(hull2.getIndexBuffer().size()==hull.getIndexBuffer().size());
			auto hull3 = std::move(hull);
			assert(hull.getIndexBuffer().size()==0);
			
			// Test 1.1 : Same test, but using the original indices.
			hull = qh.getConvexHull(pc,true,true);
			assert(hull.getIndexBuffer().size()==3*2*6);
			assert(hull.getVertexBuffer().size()==pc.size());
			assert(&(hull.getVertexBuffer()[0])==&(pc[0]));
			
			// Test 2 : random N points from the boundary of unit sphere. Result mesh must have exactly N points.
			pc = createSphere<FloatType>(1, 50);
			hull = qh.getConvexHull(pc,true,false);
			assert(pc.size() == hull.getVertexBuffer().size());
			hull = qh.getConvexHull(pc,true,true);
			// Add every vertex twice. This should not affect final mesh
			auto s = pc.size();
			for (size_t i=0;i<s;i++) {
				const auto& p = pc[i];
				pc.push_back(p);
			}
			hull = qh.getConvexHull(pc,true,false);
			assert(pc.size()/2 == hull.getVertexBuffer().size());
			
			// Test 2.1 : Multiply x components of the unit sphere vectors by a huge number => essentially we get a line
			const FloatType mul = 2*2*2;
			while (true) {
				for (auto& p : pc) {
					p.x *= mul;
				}
				hull = qh.getConvexHull(pc,true,false);
				if (hull.getVertexBuffer().size() == 4) {
					break;
				}
			}
			
			// Test 3: 0D
			pc.clear();
			vec3 centerPoint(2,2,2);
			pc.push_back(centerPoint);
			for (size_t i=0;i<100;i++) {
				auto newp = centerPoint + vec3(rnd(-0.000001f,0.000001f),rnd(-0.000001f,0.000001f),rnd(-0.000001f,0.000001f));
				pc.push_back(newp);
			}
			hull = qh.getConvexHull(pc,true,false);
			assert(hull.getIndexBuffer().size()==12);
			
			// Test 4: 2d degenerate case
			testPlanarCase();
			
			// Test 5: first a planar circle, then make a cylinder out of it
			pc.clear();
			for (size_t i=0;i<N;i++) {
				const FloatType alpha = (FloatType)i/N*2*3.14159f;
				pc.emplace_back(std::cos(alpha),0,std::sin(alpha));
			}
			hull = qh.getConvexHull(pc,true,false);
			
			assert(hull.getVertexBuffer().size() == pc.size());
			for (size_t i=0;i<N;i++) {
				pc.push_back(pc[i] + vec3(0,1,0));
			}
			hull = qh.getConvexHull(pc,true,false);
			assert(hull.getVertexBuffer().size() == pc.size());
			assert(hull.getIndexBuffer().size()/3 == pc.size()*2-4);
			
			// Test 6
			for (int x=0;;x++) {
				pc.clear();
				const FloatType l = 1;
				const FloatType r = l/(std::pow(10, x));
				for (size_t i=0;i<N;i++) {
					vec3 p = vec3(1,0,0)*i*l/(N-1);
					FloatType a = rnd(0,2*3.1415f);
					vec3 d = vec3(0,std::sin(a),std::cos(a))*r;
					pc.push_back(p+d);
				}
				hull = qh.getConvexHull(pc,true,false);
				if (hull.getVertexBuffer().size()==4) {
					break;
				}
			}
			
			// Test 7
			for (int h=0;h<100;h++) {
				pc.clear();
				const vec3 v1(rnd(-1,1),rnd(-1,1),rnd(-1,1));
				const vec3 v2(rnd(-1,1),rnd(-1,1),rnd(-1,1));
				pc.push_back(v1);
				pc.push_back(v2);
				for (int i=0;i<N;i++) {
					auto t1 = rnd(0,1);
					auto t2 = rnd(0,1);
					pc.push_back(t1*v1 + t2*v2);
				}
				hull = qh.getConvexHull(pc,true,false);
			}
			
			// Other tests
			testPlanes();
			sphereTest();
			testVector3();
			testHalfEdgeOutput();
			std::cout << "QuickHull tests succesfully passed." << std::endl;
		}
		
	}
}
