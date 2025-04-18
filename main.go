package main

import (
	"container/heap"
	"fmt"
	"image/color"
	"math"
	"math/rand"
	"net/http"
	"time"

	"github.com/fogleman/gg"
	"github.com/gin-gonic/gin"
)

type Position struct {
	X, Y int `json:"X"`
}

type Node struct {
	Pos      Position
	Cost     int
	Priority int
	Parent   *Node
	Index    int
}

type PriorityQueue []*Node

func (pq PriorityQueue) Len() int           { return len(pq) }
func (pq PriorityQueue) Less(i, j int) bool { return pq[i].Priority < pq[j].Priority }
func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].Index = i
	pq[j].Index = j
}
func (pq *PriorityQueue) Push(x interface{}) {
	node := x.(*Node)
	node.Index = len(*pq)
	*pq = append(*pq, node)
}
func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	node := old[n-1]
	node.Index = -1
	*pq = old[0 : n-1]
	return node
}

// ข้อมูลตำแหน่งบูธจริง
var predefinedBooths = map[string]Position{
	"Booth-001": {X: 2, Y: 3},
	"Booth-002": {X: 4, Y: 5},
	"Booth-003": {X: 7, Y: 8},
	"Booth-004": {X: 10, Y: 10},
	"Booth-005": {X: 15, Y: 12},
	"Booth-006": {X: 18, Y: 25},
	// เพิ่มบูธอื่น ๆ ตามข้อมูลจริง
}

func generateGrid(rows, cols int) [][]int {
	grid := make([][]int, rows)
	for i := 0; i < rows; i++ {
		grid[i] = make([]int, cols)
	}
	return grid
}

func placeBooths(grid [][]int, predefinedBooths map[string]Position) {
	for boothName, boothPos := range predefinedBooths {
		if boothPos.X >= 0 && boothPos.Y >= 0 && boothPos.X < len(grid) && boothPos.Y < len(grid[0]) {
			grid[boothPos.X][boothPos.Y] = 1 // booth
			// คุณสามารถเก็บชื่อบูธในที่นี้ได้ถ้าต้องการ
			fmt.Printf("Booth: %s at Position: (%d, %d)\n", boothName, boothPos.X, boothPos.Y)
		}
	}
}

func heuristic(a, b Position) int {
	return int(math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y)))
}

func neighbors(pos Position, grid [][]int) []Position {
	dirs := []Position{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
	var result []Position
	rows, cols := len(grid), len(grid[0])
	for _, d := range dirs {
		nx, ny := pos.X+d.X, pos.Y+d.Y
		if nx >= 0 && ny >= 0 && nx < rows && ny < cols && grid[nx][ny] == 0 {
			result = append(result, Position{nx, ny})
		}
	}
	return result
}

func findPath(grid [][]int, start, goal Position) []Position {
	openSet := make(PriorityQueue, 0)
	heap.Init(&openSet)

	startNode := &Node{Pos: start, Cost: 0, Priority: heuristic(start, goal)}
	heap.Push(&openSet, startNode)

	costSoFar := map[Position]int{start: 0}

	for openSet.Len() > 0 {
		current := heap.Pop(&openSet).(*Node)
		if current.Pos == goal {
			path := []Position{}
			for current != nil {
				path = append([]Position{current.Pos}, path...)
				current = current.Parent
			}
			return path
		}
		for _, next := range neighbors(current.Pos, grid) {
			newCost := costSoFar[current.Pos] + 1
			if c, ok := costSoFar[next]; !ok || newCost < c {
				costSoFar[next] = newCost
				priority := newCost + heuristic(next, goal)
				node := &Node{Pos: next, Cost: newCost, Priority: priority, Parent: current}
				heap.Push(&openSet, node)
			}
		}
	}
	return nil
}

func getFreePosition(grid [][]int) Position {
	for {
		x := rand.Intn(len(grid))
		y := rand.Intn(len(grid[0]))
		if grid[x][y] == 0 {
			return Position{x, y}
		}
	}
}

func drawGridImage(grid [][]int, path []Position, start, goal Position, filename string) {
	const cellSize = 20
	width := len(grid[0]) * cellSize
	height := len(grid) * cellSize
	dc := gg.NewContext(width, height)

	for i := 0; i < len(grid); i++ {
		for j := 0; j < len(grid[0]); j++ {
			switch grid[i][j] {
			case 1:
				dc.SetColor(color.RGBA{100, 100, 100, 255})
			default:
				dc.SetColor(color.White)
			}
			dc.DrawRectangle(float64(j*cellSize), float64(i*cellSize), cellSize, cellSize)
			dc.Fill()
		}
	}

	dc.SetColor(color.RGBA{0, 0, 255, 255})
	for _, p := range path {
		dc.DrawRectangle(float64(p.Y*cellSize), float64(p.X*cellSize), cellSize, cellSize)
		dc.Fill()
	}
	dc.SetColor(color.RGBA{0, 200, 0, 255})
	dc.DrawRectangle(float64(start.Y*cellSize), float64(start.X*cellSize), cellSize, cellSize)
	dc.Fill()
	dc.SetColor(color.RGBA{255, 0, 0, 255})
	dc.DrawRectangle(float64(goal.Y*cellSize), float64(goal.X*cellSize), cellSize, cellSize)
	dc.Fill()

	dc.SavePNG(filename)
}

func main() {
	rand.Seed(time.Now().UnixNano())
	router := gin.Default()
	router.Static("/static", ".")

	router.GET("/path", func(c *gin.Context) {
		rows, cols := 20, 30
		grid := generateGrid(rows, cols)

		// วางตำแหน่งบูธจากข้อมูลจริง
		placeBooths(grid, predefinedBooths)

		// กำหนดจุดเริ่มต้นและจุดปลายทาง
		start := getFreePosition(grid) // หรือใช้ข้อมูลที่ผู้ใช้กำหนด
		goal := getFreePosition(grid)  // หรือใช้ข้อมูลที่ผู้ใช้กำหนด

		// หาทาง
		path := findPath(grid, start, goal)

		// สร้างภาพแผนที่
		filename := fmt.Sprintf("map_%d.png", time.Now().UnixNano())
		drawGridImage(grid, path, start, goal, filename)

		// ส่งข้อมูลผลลัพธ์กลับ
		c.JSON(http.StatusOK, gin.H{
			"start": start,
			"goal":  goal,
			"path":  path,
			"map":   "/static/" + filename,
			"booths": predefinedBooths, // ส่งข้อมูลบูธทั้งหมดกลับด้วย
		})
	})

	router.Run(":8080")
}
