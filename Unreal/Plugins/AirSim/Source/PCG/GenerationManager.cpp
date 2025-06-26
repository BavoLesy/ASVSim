// Fill out your copyright notice in the Description page of Project Settings.


#include "PCG/GenerationManager.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetMathLibrary.h"
#include "Math/Transform.h"

// Sets default values
AGenerationManager::AGenerationManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

}

// Called when the game starts or when spawned
void AGenerationManager::BeginPlay()
{
	Super::BeginPlay();
	
	aseed = 10;
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d"), aseed);
}

bool AGenerationManager::HavenStep(TArray<FTransform>& haven, float mina, float maxa, float mind, float maxd)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 2"), aseed);
	return true;
}
void AGenerationManager::WallStep(TArray<FTransform>& haven, TArray<FTransform>& wall, float min, float max)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 3"), aseed);
}
void AGenerationManager::GenerateWalls(TArray<FTransform>& points, bool inlets, float min, float max)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 4"), aseed);
}
void AGenerationManager::InletFunction(FTransform begin, FTransform end, FRotator rotation, float min, float max, float length)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 5"), aseed);
}
void AGenerationManager::InletGeneration(TArray<FTransform>& points)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 6"), aseed);
}
void AGenerationManager::Clear()
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 6"), aseed);
}
void AGenerationManager::SetSeed(int32 seed)
{
	aseed = seed;
	stream.Initialize(aseed);
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 7"), aseed);
}
void AGenerationManager::generateTerrain(FString type, float mina, float maxa, float mind, float maxd, int length)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 8"), aseed);
}
void AGenerationManager::getGoal(int32 distance, FVector& location, FVector& left, FVector& right)
{
	UE_LOG(LogTemp, Warning, TEXT("MyNumber is now: %d  Part 9"), aseed);
}

// Called every frame
void AGenerationManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

