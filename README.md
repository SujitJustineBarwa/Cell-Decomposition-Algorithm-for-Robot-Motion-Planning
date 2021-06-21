# Cell Decomposition Algorithm for Robot Motion Planning
 Cell Decomposition algorithm is a technique in which we decompose the workspace (2D Shape) into quadilateral Cells which are then treated as nodes and using BFS or DFS algorithm we can find the path from Start to End.
 In the provided Code,two methods of doing this is shown Naive and Sweep strategy.The sweep is faster than the naive approach.
 Both the Strategies are implemented and are commented out well.
 Special Thanks to Atul Thakur Sir(from IIT Patna).
 
 # Problem Statement :
\
A.Take input from user using ginput command in matlab(take input(as point)from matlab plot when you click in it.Use these input these plots to obstacles and workspace.\
B.Write a condition to ensure that the X coordinates of the vertices are unique.\
C.Perform cell decomposition using Naïve approach. Store the cells in a suitable datastructure.\
D.Improve the cell decomposition code with sweeping algorithm.\
E.Compare the time taken for cell decomposition suing both Naïve and sweeping algorithm approaches using tic and toc in matlab.

# The approach :
\
**Task 1 :**
\
Ensure all x are unique
The array X stores the x-coordinate of all the vertices.
if length of X is not equal to length of unique(X) then stop.
\

**Task 2 :** 
\
Naiive Strategy
Cell Decomposition using Naiive Strategy:-
All the edges are stored in all_edges as structure array,each  element containing the initial and final coordinate points.
A ployshape(pygon) is made as a configuration space minus obstacle.
A loop is started moving through the X array (loop1).
Below picture show the Edges of a sample input:
<p align="center">
  <img src="https://github.com/SujitJustineBarwa/Cell-Decomposition-Algorithm-for-Robot-Motion-Planning/blob/main/Images/Edges%20graph.png" />
</p>

**a) Collecting the intersection points :**
A  2nd loop starts moving through all_edges to check the edges intersecting vertical extension 
A Table T stores all the points and the edges that are connected to the current extension respectively.
The 2nd loop is stopped
Note : these two loops are what make this whole program an O(n^2) time complex.

**b) Processing on T :**
The point immediate upper to current vertices and immediate below to current vertices are kept and everything else is deleted.
The upper mid point and lower mid point are calculated by taking the average of upper intersection and lower intersection.
Isinterior() function is used to determine if the mid points are inside pygon i.e. the Config. Space.



**c) Cell making Process :**
A Table T_Manager is employed as data structure  to manage vertices and edges.It is updated as :
 		T_Manager = [T_Manager;T]

This table has some internal operations to do in the following sequence :
	Transfer all the rows having equal edges to an Temp_T.
	Eliminate all the points having a equal edges.
	Re-adding T.
	Eliminating all the edges having initial and final points on left of x.

**d) Converting the Temp_T to cell :**
It should be kept in mind that the from Temp_two cells can form simlatenously.
The picture below depicts the this case.
<p align="center">
  <img src="https://github.com/SujitJustineBarwa/Cell-Decomposition-Algorithm-for-Robot-Motion-Planning/blob/main/Images/img3.PNG" />
</p>
Let the point in red circle be x7,y7 so the upper blue point will be x7,y_upper and below would be x7,y_down.Next lets say the green point where the 3 lines are meeting which is actually vertex of the obstacle be x4,y4 and the upper green point would be x4,y_upper(Note : x4,y_upper and x7,y_upper lies in the same edge).Same with the maroon circles,Assumed x6,y6 and x6,y_down.

\
The Following 2 steps are followed to performing the segregation completely :- 
     	 
**Step 1**
First the table is sorted on the basis of x column.
Edges of first two rows are searched on the table and respective X and Y Co-ordinates are transferred to cell_vertices array .

<table style="width: 796px; height: 230px;">
<tbody>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;X Co-ordinate of the Cell Vertex</td>
<td style="width: 403.417px; height: 25px;">&nbsp;Y Co-ordinate of the cell Vertex</td>
<td style="width: 224.1px; height: 25px;">&nbsp;The Edge on which the current Point Lies in</td>
</tr>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;x4</td>
<td style="width: 403.417px; height: 25px;">&nbsp;y4_upper</td>
<td style="width: 224.1px; height: 25px;">&nbsp;e2</td>
</tr>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;x4</td>
<td style="width: 403.417px; height: 25px;">&nbsp;y4</td>
<td style="width: 224.1px; height: 25px;">&nbsp;e6</td>
</tr>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;x6</td>
<td style="width: 403.417px; height: 25px;">&nbsp;y6</td>
<td style="width: 224.1px; height: 25px;">&nbsp;e7</td>
</tr>
<tr style="height: 25.9333px;">
<td style="width: 167.683px; height: 25.9333px;">&nbsp;x6</td>
<td style="width: 403.417px; height: 25.9333px;">&nbsp;y6_down</td>
<td style="width: 224.1px; height: 25.9333px;">&nbsp;e12</td>
</tr>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;x7</td>
<td style="width: 403.417px; height: 25px;">&nbsp;y7</td>
<td style="width: 224.1px; height: 25px;">&nbsp;e6</td>
</tr>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;x7</td>
<td style="width: 403.417px; height: 25px;">&nbsp;y7</td>
<td style="width: 224.1px; height: 25px;">&nbsp;e7</td>
</tr>
<tr style="height: 25px;">
<td style="width: 167.683px; height: 25px;">&nbsp;x7</td>
<td style="width: 403.417px; height: 25px;">&nbsp;y7_upper</td>
<td style="width: 224.1px; height: 25px;">&nbsp;e2</td>
</tr>
<tr style="height: 24px;">
<td style="width: 167.683px; height: 24px;">&nbsp;x7</td>
<td style="width: 403.417px; height: 24px;">&nbsp;y7_down</td>
<td style="width: 224.1px; height: 24px;">e12</td>
</tr>
</tbody>
</table>
<!-- DivTable.com -->
<p>&nbsp;</p>
 
 
 Cell 1 :
<!-- DivTable.com -->
<p>&nbsp;</p>
<table style="height: 144px;" width="805">
<tbody>
<tr>
<td style="width: 171.333px;">x4</td>
<td style="width: 455.317px;">y4_upper</td>
<td style="width: 177.55px;">e2</td>
</tr>
<tr>
<td style="width: 171.333px;">x4</td>
<td style="width: 455.317px;">y4</td>
<td style="width: 177.55px;">e6</td>
</tr>
<tr>
<td style="width: 171.333px;">x7</td>
<td style="width: 455.317px;">y7</td>
<td style="width: 177.55px;">e6</td>
</tr>
<tr>
<td style="width: 171.333px;">x7</td>
<td style="width: 455.317px;">y7_upper</td>
<td style="width: 177.55px;">e2</td>
</tr>
</tbody>
</table>
<p>&nbsp;</p>

Cell 2:
<!-- DivTable.com -->
<p>&nbsp;</p>
<table style="height: 144px;" width="805">
<tbody>
<tr>
<td style="width: 171.333px;">x6</td>
<td style="width: 455.317px;">y6_upper</td>
<td style="width: 177.55px;">e7</td>
</tr>
<tr>
<td style="width: 171.333px;">x6</td>
<td style="width: 455.317px;">y6</td>
<td style="width: 177.55px;">e12</td>
</tr>
<tr>
<td style="width: 171.333px;">x7</td>
<td style="width: 455.317px;">y7</td>
<td style="width: 177.55px;">e7</td>
</tr>
<tr>
<td style="width: 171.333px;">x7</td>
<td style="width: 455.317px;">y7_down</td>
<td style="width: 177.55px;">e12</td>
</tr>
</tbody>
</table>
<p>&nbsp;</p>

**Step 2**

If cell _vertices array length is 4 they are transferred to a cells array and cell_vertices is reset.
Cells array is datatype cell which stores 4 coordinates at each of its index.

